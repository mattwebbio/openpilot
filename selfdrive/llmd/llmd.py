import os, time, json, base64
import numpy as np, cv2
from ollama import Client
from cereal.messaging import SubMaster, PubMaster
from msgq.visionipc import VisionIpcClient, VisionStreamType
import cereal.messaging as messaging

def convert_nv12_to_jpg(buf_main, jpg_quality=80, scale=1):
  y_size = buf_main.stride * buf_main.height
  y_data = np.frombuffer(buf_main.data[:y_size], dtype=np.uint8).reshape(buf_main.height, buf_main.stride)[:, :buf_main.width]
  uv_height = buf_main.height // 2
  uv_size = buf_main.stride * uv_height
  uv_data = np.frombuffer(buf_main.data[buf_main.uv_offset:buf_main.uv_offset + uv_size], dtype=np.uint8).reshape(uv_height, buf_main.stride)[:, :buf_main.width]
  nv12_buffer = np.zeros(buf_main.height * buf_main.width + uv_height * buf_main.width, dtype=np.uint8)
  nv12_buffer[:buf_main.height * buf_main.width] = y_data.reshape(-1)
  nv12_buffer[buf_main.height * buf_main.width:] = uv_data.reshape(-1)
  nv12_image = nv12_buffer.reshape(buf_main.height + uv_height, buf_main.width)
  bgr_image = cv2.cvtColor(nv12_image, cv2.COLOR_YUV2BGR_NV12)
  if scale < 1.0:
    new_width = int(buf_main.width * scale)
    new_height = int(buf_main.height * scale)
    bgr_image = cv2.resize(bgr_image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
  success, jpg_data = cv2.imencode('.jpg', bgr_image, [cv2.IMWRITE_JPEG_QUALITY, jpg_quality])
  return jpg_data.tobytes() if success else None


class OllamaClient:
  def __init__(self, include_reason=True):
    self.model = "llava:7b"
    self.include_reason = include_reason
    self.client = None
    self.host = None
    self._connect()

  def _connect(self):
    """Try to connect to Ollama, starting with localhost, then fallback to 10.78.234.85"""
    hosts = ["http://localhost:11434", "http://10.78.234.85:11434"]
    for host in hosts:
      try:
        client = Client(host=host)
        # Test connection by making a simple request
        client.list()
        self.client = client
        self.host = host
        print(f"Connected to Ollama at {host}")
        return
      except Exception as e:
        print(f"Failed to connect to {host}: {e}")
    raise ConnectionError("Failed to connect to Ollama on any available host")

  def query(self, prompt, image_paths=None):
    images = []
    if image_paths:
      paths = image_paths if isinstance(image_paths, list) else [image_paths]
      for path in paths:
        if path and os.path.exists(path):
          images.append(base64.b64encode(open(path, 'rb').read()).decode('utf-8'))
    response = self.client.generate(model=self.model, prompt=prompt, images=images or None, stream=False)
    return (response.get('response') if isinstance(response, dict) else response.response).strip()


def main():
  client = OllamaClient()
  vipc = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, conflate=True)
  while not vipc.connect(False): time.sleep(0.1)

  sm = SubMaster(['carState'])
  pm = PubMaster(['llmDecision'])
  frame_count = 0
  speed_history = []

  while True:
    sm.update(1000)
    image_buf = vipc.recv()
    if image_buf is None or not sm.updated['carState']: continue

    frame_count += 1

    car_state = sm['carState']
    img_path = f"llmd_frame_{frame_count}.jpg"
    jpg = convert_nv12_to_jpg(image_buf)

    if not jpg: continue
    with open(img_path, 'wb') as f:
      f.write(jpg)

    try:
      json_ex = '{"predicted_speed": <float_m_s>, "reason": "... (max. 10 words)"}' if client.include_reason else '{"predicted_speed": <float_m_s>}'
      hist_ctx = f"Recent speeds: {speed_history[-5:]}\nMaintain consistency.\n" if len(speed_history) != 0 else ""

      try:
        cruise = car_state.car_cruise_speed
      except Exception:
        cruise = car_state.cruiseState.speed

      prompt = f"""Predict safe driving speed based on road conditions.
Current speed: {car_state.vEgo:.1f} m/s, cruise setting: {cruise:.1f} m/s.
{hist_ctx}
Road type and speed guidance:
- Highway/Freeway (straight, open): 28-32 m/s
- Urban/City (streets, intersections): 12-18 m/s
- Pedestrian zones or construction: Reduce to 5-12 m/s
Never predict below 5 m/s unless severe hazard. Never increase speed when hazards detected.
Slow down for curves slightly.
Return ONLY valid JSON: {json_ex}"""

      resp = client.query(prompt, image_paths=img_path).strip()
      if resp.startswith("```"): resp = resp.split("```")[1]; resp = resp[4:] if resp.startswith("json") else resp
      resp = resp.replace('\\_', '_').replace('\\n', ' ').replace('\n', ' ')
      j_start, j_end = resp.find('{'), resp.rfind('}')
      if j_start != -1 and j_end != -1: resp = resp[j_start:j_end+1]
      decision = json.loads(resp)

      print(decision, frame_count)

      if 'predicted_speed' not in decision: continue
      pred_speed = decision.get('predicted_speed', car_state.cruiseState.speed)

      speed_history.append(pred_speed)
      if len(speed_history) > 4:
        speed_history.pop(0)

      msg = messaging.new_message('llmDecision')
      msg.llmDecision.predictedSpeed = float(pred_speed)
      msg.llmDecision.reason = decision.get('reason', '')
      msg.llmDecision.frame = frame_count
      pm.send('llmDecision', msg)
    except (ValueError, KeyError, json.JSONDecodeError): pass
    finally:
      try: os.remove(img_path)
      except OSError: pass

if __name__ == "__main__":
  main()
