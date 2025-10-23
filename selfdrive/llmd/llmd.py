import os
import time
import base64
import json
import anthropic
from cereal.messaging import SubMaster, PubMaster
from msgq.visionipc import VisionIpcClient, VisionStreamType
from cereal import log


def main():
  # Initialize Claude API client
  api_key = os.getenv("ANTHROPIC_API_KEY")
  if not api_key:
    raise ValueError("ANTHROPIC_API_KEY environment variable not set")

  client = anthropic.Anthropic(api_key=api_key)

  # Initialize camera clients
  vipc_main = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, conflate=True)
  vipc_extra = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, conflate=True)

  # Connect to cameras
  while not vipc_main.connect(False):
    time.sleep(0.1)

  while not vipc_extra.connect(False):
    time.sleep(0.1)

  print("Camera connections established")

  # Subscribe to carState for vehicle telemetry
  sm = SubMaster(['carState'])
  pm = PubMaster(['llmDecision'])

  # Main loop
  frame_count = 0
  while True:
    response_text = None
    try:
      sm.update()

      # Receive frames from both cameras
      buf_main = vipc_main.recv()
      buf_extra = vipc_extra.recv()

      if buf_main is None or buf_extra is None:
        continue

      if not sm.updated['carState']:
        continue

      frame_count += 1

      # Get vehicle data from carState
      car_state = sm['carState']
      current_speed = car_state.vEgo  # m/s
      cruise_speed = car_state.cruiseState.speed  # m/s

      print(f"\n[Frame {frame_count}] Speed: {current_speed:.1f} m/s, Cruise: {cruise_speed:.1f} m/s")

      # Convert buffers to base64 for API
      main_image_b64 = base64.standard_b64encode(buf_main.data).decode("utf-8")
      extra_image_b64 = base64.standard_b64encode(buf_extra.data).decode("utf-8")

      # Create prompt with vehicle telemetry
      prompt = f"""I am driving at {current_speed:.1f} m/s with cruise control set to {cruise_speed:.1f} m/s.
Based on the road conditions and traffic in these images, should I do a lane change?

Respond with ONLY a JSON object (no markdown, no extra text) with this exact format:
{{
  "should_change_lane": true/false,
  "direction": "left"/"right"/"none",
  "confidence": 0.0-1.0,
  "reason": "brief explanation"
}}"""

      # Send to Claude API with both images
      message = client.messages.create(
        model="claude-3-5-sonnet-20241022",
        max_tokens=256,
        messages=[
          {
            "role": "user",
            "content": [
              {
                "type": "text",
                "text": prompt,
              },
              {
                "type": "image",
                "source": {
                  "type": "base64",
                  "media_type": "image/raw",
                  "data": main_image_b64,
                },
              },
              {
                "type": "text",
                "text": "Wide camera view:",
              },
              {
                "type": "image",
                "source": {
                  "type": "base64",
                  "media_type": "image/raw",
                  "data": extra_image_b64,
                },
              },
            ],
          }
        ],
      )

      # Parse Claude's response as JSON
      response_text = message.content[0].text.strip()

      # Remove markdown code blocks if present
      if response_text.startswith("```"):
        response_text = response_text.split("```")[1]
        if response_text.startswith("json"):
          response_text = response_text[4:]
        response_text = response_text.strip()

      decision = json.loads(response_text)

      print(f"Claude Decision: {decision}")

      # Map direction to Custom.LlmDecision.LaneChangeDirection
      direction_map = {
        "left": log.LlmDecision.LaneChangeDirection.left,
        "right": log.LlmDecision.LaneChangeDirection.right,
        "none": log.LlmDecision.LaneChangeDirection.none,
      }

      # Create and send llmDecision message
      msg = log.Event.new_message()
      msg.llmDecision.shouldChangeLane = decision.get('should_change_lane', False)
      msg.llmDecision.direction = direction_map.get(decision.get('direction', 'none'), log.LlmDecision.LaneChangeDirection.none)
      msg.llmDecision.confidence = float(decision.get('confidence', 0.0))
      msg.llmDecision.reason = decision.get('reason', '')

      pm.send('llmDecision', msg)

    except json.JSONDecodeError as e:
      print(f"JSON Parse Error: {e}")
      if response_text:
        print(f"Response was: {response_text}")
    except anthropic.APIError as e:
      print(f"API Error: {e}")
    except Exception as e:
      print(f"Error: {e}")
      time.sleep(1)


if __name__ == "__main__":
  main()
