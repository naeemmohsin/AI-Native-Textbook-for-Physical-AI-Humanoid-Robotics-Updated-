#!/usr/bin/env python3
"""
Whisper Speech-to-Text ROS 2 Node

Subscribes to raw audio and publishes transcriptions.
Uses local Whisper model for low-latency inference.

Topics:
    Subscribed: /voice/audio_raw (audio_common_msgs/AudioData)
    Published: /voice/transcript (std_msgs/String)

Parameters:
    model_size: Whisper model size (tiny, base, small, medium, large-v3)
    language: Language code for transcription (en, es, fr, etc.)
    sample_rate: Audio sample rate in Hz (default: 16000)
    buffer_seconds: Maximum audio buffer length
    silence_threshold: RMS threshold for speech detection
    min_audio_length: Minimum speech duration to process

Usage:
    ros2 run whisper_node whisper_node --ros-args -p model_size:=base -p language:=en
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import numpy as np
import threading
from collections import deque
from typing import Optional

# Whisper import with fallback
try:
    import whisper
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False
    print("Warning: whisper not installed. Install with: pip install openai-whisper")

# Optional: faster-whisper for better performance
try:
    from faster_whisper import WhisperModel
    FASTER_WHISPER_AVAILABLE = True
except ImportError:
    FASTER_WHISPER_AVAILABLE = False


class WhisperNode(Node):
    """ROS 2 node for speech-to-text using OpenAI Whisper."""

    def __init__(self):
        super().__init__('whisper_node')

        # Declare parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('language', 'en')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('buffer_seconds', 5.0)
        self.declare_parameter('silence_threshold', 0.01)
        self.declare_parameter('min_audio_length', 0.5)
        self.declare_parameter('use_faster_whisper', False)
        self.declare_parameter('device', 'cpu')  # 'cpu' or 'cuda'
        self.declare_parameter('compute_type', 'int8')  # For faster-whisper

        # Get parameters
        model_size = self.get_parameter('model_size').value
        self.language = self.get_parameter('language').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.buffer_seconds = self.get_parameter('buffer_seconds').value
        self.silence_threshold = self.get_parameter('silence_threshold').value
        self.min_audio_length = self.get_parameter('min_audio_length').value
        use_faster = self.get_parameter('use_faster_whisper').value
        device = self.get_parameter('device').value
        compute_type = self.get_parameter('compute_type').value

        # Load Whisper model
        self.model = None
        self.use_faster_whisper = False

        if use_faster and FASTER_WHISPER_AVAILABLE:
            self.get_logger().info(f'Loading faster-whisper model: {model_size}')
            self.model = WhisperModel(
                model_size,
                device=device,
                compute_type=compute_type
            )
            self.use_faster_whisper = True
        elif WHISPER_AVAILABLE:
            self.get_logger().info(f'Loading Whisper model: {model_size}')
            self.model = whisper.load_model(model_size, device=device)
        else:
            self.get_logger().error('No Whisper implementation available!')
            raise RuntimeError("Install whisper or faster-whisper")

        self.get_logger().info('Whisper model loaded successfully')

        # Audio buffer for speech detection
        buffer_samples = int(self.buffer_seconds * self.sample_rate)
        self.audio_buffer = deque(maxlen=buffer_samples)

        # Speech state tracking
        self.is_speaking = False
        self.speech_buffer: list = []
        self.silence_frames = 0
        self.silence_frames_threshold = int(0.5 * self.sample_rate / 512)  # 0.5s silence

        # Thread lock for buffer access
        self.buffer_lock = threading.Lock()

        # Transcription state
        self.is_transcribing = False

        # Create subscriber and publisher
        self.audio_sub = self.create_subscription(
            AudioData,
            '/voice/audio_raw',
            self.audio_callback,
            10
        )

        self.transcript_pub = self.create_publisher(
            String,
            '/voice/transcript',
            10
        )

        # Processing timer (check for complete utterances)
        self.create_timer(0.1, self.process_audio)

        self.get_logger().info('Whisper node initialized and ready')

    def audio_callback(self, msg: AudioData) -> None:
        """Handle incoming audio data."""
        # Convert bytes to numpy array (assuming 16-bit signed audio)
        audio_data = np.frombuffer(msg.data, dtype=np.int16)
        audio_float = audio_data.astype(np.float32) / 32768.0

        with self.buffer_lock:
            # Calculate RMS for speech activity detection
            rms = np.sqrt(np.mean(audio_float ** 2))

            if rms > self.silence_threshold:
                # Speech detected
                self.silence_frames = 0
                if not self.is_speaking:
                    self.is_speaking = True
                    self.get_logger().debug('Speech started')
                self.speech_buffer.extend(audio_float)
            else:
                # Silence detected
                if self.is_speaking:
                    self.silence_frames += 1
                    self.speech_buffer.extend(audio_float)  # Include trailing silence

                    if self.silence_frames >= self.silence_frames_threshold:
                        # End of speech
                        self.is_speaking = False
                        self.get_logger().debug('Speech ended')

    def process_audio(self) -> None:
        """Process accumulated speech and transcribe."""
        with self.buffer_lock:
            if not self.is_speaking and len(self.speech_buffer) > 0 and not self.is_transcribing:
                audio_length = len(self.speech_buffer) / self.sample_rate

                if audio_length >= self.min_audio_length:
                    # Copy buffer and clear
                    audio_array = np.array(self.speech_buffer, dtype=np.float32)
                    self.speech_buffer = []
                    self.is_transcribing = True

                    # Run transcription in separate thread
                    threading.Thread(
                        target=self.transcribe,
                        args=(audio_array,),
                        daemon=True
                    ).start()
                else:
                    # Too short, discard
                    self.speech_buffer = []
                    self.get_logger().debug('Audio too short, discarding')

    def transcribe(self, audio: np.ndarray) -> None:
        """Transcribe audio array using Whisper."""
        try:
            text = self._run_transcription(audio)

            if text:
                self.get_logger().info(f'Transcription: "{text}"')

                msg = String()
                msg.data = text
                self.transcript_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')
        finally:
            self.is_transcribing = False

    def _run_transcription(self, audio: np.ndarray) -> Optional[str]:
        """Run the actual transcription based on backend."""
        if self.use_faster_whisper:
            segments, info = self.model.transcribe(
                audio,
                language=self.language,
                beam_size=5,
                vad_filter=True
            )
            text = " ".join([segment.text for segment in segments])
        else:
            result = self.model.transcribe(
                audio,
                language=self.language,
                fp16=False,
                task='transcribe'
            )
            text = result['text']

        return text.strip() if text else None


def main(args=None):
    rclpy.init(args=args)

    try:
        node = WhisperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
