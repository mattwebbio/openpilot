import os
import time
import json
import subprocess
import tempfile
from cereal.messaging import SubMaster, PubMaster
from msgq.visionipc import VisionIpcClient, VisionStreamType
from cereal import log


def call_claude_cli(prompt, image_paths):
  """Call Claude via CLI subprocess with images"""
  try:
    # Build the claude command with images
    cmd = ["/data/npm-global/bin/claude"]

    # Add images to the command
    for img_path in image_paths:
      cmd.extend(["-i", img_path])

    # Set up environment with API key
    env = os.environ.copy()
    env["CLAUDE_CODE_OAUTH_TOKEN"] = "PLACEHOLDER"

    # Run the command with the prompt piped to stdin
    result = subprocess.run(
      cmd,
      input=prompt.encode('utf-8'),
      capture_output=True,
      timeout=30,
      env=env
    )

    if result.returncode != 0:
      raise RuntimeError(f"Claude CLI failed: {result.stderr.decode('utf-8')}")

    return result.stdout.decode('utf-8').strip()
  except Exception as e:
    raise RuntimeError(f"Failed to call Claude CLI: {str(e)}")


def main():
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

      # Create temporary files for the images
      with tempfile.TemporaryDirectory() as tmpdir:
        main_img_path = os.path.join(tmpdir, "main_camera.raw")
        extra_img_path = os.path.join(tmpdir, "wide_camera.raw")

        # Write image data to temporary files
        with open(main_img_path, 'wb') as f:
          f.write(buf_main.data)
        with open(extra_img_path, 'wb') as f:
          f.write(buf_extra.data)

        # Create prompt with vehicle telemetry
        prompt = f"""I am driving at {current_speed:.1f} m/s with cruise control set to {cruise_speed:.1f} m/s.
Based on the road conditions and traffic in these images, should I do a lane change?

Main camera view is in the first image.
Wide camera view is in the second image.

Respond with ONLY a JSON object (no markdown, no extra text) with this exact format:
{{
  "should_change_lane": true/false,
  "direction": "left"/"right"/"none",
  "confidence": 0.0-1.0,
  "reason": "brief explanation"
}}"""

        # Call Claude via CLI with both images
        response_text = call_claude_cli(prompt, [main_img_path, extra_img_path])

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
    except RuntimeError as e:
      print(f"Claude CLI Error: {e}")
      time.sleep(1)
    except Exception as e:
      print(f"Error: {e}")
      time.sleep(1)


if __name__ == "__main__":
  main()
