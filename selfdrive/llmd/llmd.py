import os
import time
import json
import base64
import numpy as np
import cv2
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
  if not success:
    raise RuntimeError("Failed to encode image")
  return jpg_data.tobytes()


class OllamaClient:
  def __init__(self, host="http://localhost:11434", include_reason=True):
    self.model = "llava:7b"
    self.client = Client(host=host)
    self.include_reason = include_reason

  def query(self, prompt, image_paths=None):
    images = []
    if image_paths:
      if not isinstance(image_paths, list):
        image_paths = [image_paths]
      for image_path in image_paths:
        if image_path and os.path.exists(image_path):
          try:
            with open(image_path, 'rb') as f:
              image_data = base64.b64encode(f.read()).decode('utf-8')
              images.append(image_data)
          except Exception as e:
            raise RuntimeError(f"Failed to read image file: {str(e)}")

    try:
      # Use Ollama's generate endpoint with images
      response = self.client.generate(
        model=self.model,
        prompt=prompt,
        images=images if images else None,
        stream=False
      )

      # Handle both dict and object responses from Ollama
      if isinstance(response, dict) and 'response' in response:
        return response['response'].strip()
      elif hasattr(response, 'response'):
        return response.response.strip()
      else:
        raise RuntimeError(f"Unexpected response format: {response}")

    except Exception as e:
      raise RuntimeError(f"Failed to query Ollama: {str(e)}")


def main():
  OLLAMA_HOST = "http://10.78.234.85:11434" # "http://192.168.178.65:11434"
  ollama_client = OllamaClient(host=OLLAMA_HOST)
  try:
    vipc_main = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, conflate=True)
    vipc_extra = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, conflate=True)
    vipc_driver = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_DRIVER, conflate=True)

    while not vipc_main.connect(False):
      time.sleep(0.1)
    while not vipc_extra.connect(False):
      time.sleep(0.1)
    while not vipc_driver.connect(False):
      time.sleep(0.1)

    sm = SubMaster(['carState'])
    pm = PubMaster(['llmDecision'])
    frame_count = 0
    last_decision_time = 0.0
    decision_interval = 2.0
    last_lane_change_time = time.time()

    while True:
      sm.update(1000)
      image_buf = vipc_extra.recv()
      driver_image_buf = vipc_driver.recv()

      if image_buf is None or driver_image_buf is None or not sm.updated['carState']:
        continue

      frame_count += 1

      current_time = time.time()
      if current_time - last_decision_time < decision_interval:
        continue
      last_decision_time = current_time

      car_state = sm['carState']
      current_speed = car_state.vEgo
      cruise_speed = car_state.cruiseState.speed
      left_blindspot = car_state.leftBlindspot
      right_blindspot = car_state.rightBlindspot

      jpg_data = convert_nv12_to_jpg(image_buf)
      driver_jpg_data = convert_nv12_to_jpg(driver_image_buf)
      main_img_path = f"llmd_frame_{frame_count}.jpg"
      driver_img_path = f"llmd_driver_{frame_count}.jpg"
      with open(main_img_path, 'wb') as f:
        f.write(jpg_data)
      with open(driver_img_path, 'wb') as f:
        f.write(driver_jpg_data)

      try:
        time_since_last_change = time.time() - last_lane_change_time

        blindspot_warning = ""
        if left_blindspot:
          blindspot_warning += "WARNING: Vehicle in left blind spot. "
        if right_blindspot:
          blindspot_warning += "WARNING: Vehicle in right blind spot. "

        if ollama_client.include_reason:
          json_example = '{"should_change_lane": true, "direction": "left", "confidence": 0.0 - 1.0, "reason": "explanation (max. 20 words)"}'
        else:
          json_example = '{"should_change_lane": true, "direction": "left", "confidence": 0.0 - 1.0}'

        prompt = (
          f"You have two camera views:\n"
          f"1. Forward-facing camera (wide road view) - Image 1\n"
          f"2. Driver-facing camera (side window view to detect adjacent vehicles) - Image 2\n\n"
          f"Current state: {current_speed:.1f} m/s, cruise set to {cruise_speed:.1f} m/s. "
          f"Last lane change was {time_since_last_change:.0f} seconds ago. "
          f"{blindspot_warning}\n"
          f"Favor rightmost lanes, but prioritize being with similarly-paced traffic over lane position. "
          f"Overtake only when absolutely safe and significantly impeded by slower traffic. "
          f"Treat leftward changes as risky maneuvers requiring strong justification. "
          f"Only change lanes when:\n"
          f"- Lane markings are dashed - NEVER cross solid lines\n"
          f"- There is adequate space ahead and behind\n"
          f"- The driver-facing camera (Image 2) shows NO vehicle visible through the side window for the target lane\n"
          f"- For LEFT lane change: check both forward camera (Image 1) and left side window (Image 2) for vehicles\n"
          f"- For RIGHT lane change: check both forward camera (Image 1) and right side window (Image 2) for vehicles\n"
          f"Avoid frequent changes.\n"
          f"Should I change lanes? Respond with ONLY a JSON object (no markdown, no extra text):\n"
          f"{json_example}"
        )

        try:
          response_text = ollama_client.query(prompt, image_paths=[main_img_path, driver_img_path])
        except Exception as e:
          print(f"Error querying Ollama: {e}")
          continue

        # Clean up response text
        response_text = response_text.strip()

        # Remove markdown code blocks if present
        if response_text.startswith("```"):
          response_text = response_text.split("```")[1]
          if response_text.startswith("json"):
            response_text = response_text[4:]
          response_text = response_text.strip()

        # Remove common escaping issues
        response_text = response_text.replace('\\_', '_')
        response_text = response_text.replace('\\n', ' ')
        response_text = response_text.replace('\n', ' ')

        # Extract JSON if there's extra text before/after
        json_start = response_text.find('{')
        json_end = response_text.rfind('}')
        if json_start != -1 and json_end != -1:
          response_text = response_text[json_start:json_end+1]

        # Parse JSON with error handling
        try:
          decision = json.loads(response_text)
        except json.JSONDecodeError as e:
          print(f"Failed to parse JSON response: {e}")
          print(f"Response text: {response_text[:200]}")
          continue

        # Validate decision structure
        if not all(key in decision for key in ['should_change_lane', 'direction', 'confidence']):
          print(f"Invalid decision structure: missing required keys")
          print(f"Decision: {decision}")
          continue

        print(decision)

        # Create and send llmDecision message using standard messaging API
        llm_msg = messaging.new_message('llmDecision')
        direction = decision.get('direction', 'none').lower()

        llm_msg.llmDecision.shouldChangeLane = decision.get('should_change_lane', False)
        llm_msg.llmDecision.confidence = decision.get('confidence', 0)
        llm_msg.llmDecision.reason = decision.get('reason', '')
        llm_msg.llmDecision.frame = frame_count

        confidence = decision.get('confidence', 0)

        if direction == 'left' and confidence > 0.7:
          llm_msg.llmDecision.direction = 1
        elif direction == 'right' and confidence > 0.7:
          llm_msg.llmDecision.direction = 2
        else:
          llm_msg.llmDecision.direction = 0

        if decision.get('should_change_lane', False):
          last_lane_change_time = time.time()

        pm.send('llmDecision', llm_msg)

      except Exception as e:
        print(f"Error in main loop: {e}")
      finally:
        try:
          os.remove(main_img_path)
        except OSError:
          pass
        try:
          os.remove(driver_img_path)
        except OSError:
          pass

  except KeyboardInterrupt:
    pass
  except Exception as e:
    print(f"Error: {e}")


if __name__ == "__main__":
  main()
