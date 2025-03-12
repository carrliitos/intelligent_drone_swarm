import struct
import json
import os
import threading
from pathlib import Path

from utils import logger
from utils import context

directory = context.get_context(__file__)
logger_file_name = Path(directory).stem
logger_name = Path(__file__).stem
logger = logger.setup_logger(logger_name, f"{directory}/logs/{logger_file_name}.log")


class TOCHandler:
  CMD_TOC_INFO = 0x10  # Request TOC size
  CMD_TOC_ELEMENT = 0x11  # Request individual TOC element
  TOC_CACHE_FILE = f"{directory}/cache/toc_cache.json"

  def __init__(self, udp_connection):
    self.udp_connection = udp_connection
    self.toc = {}
    self.toc_size = None
    self.ready = threading.Event()

  def fetch_toc(self):
    """Fetch the TOC from cache or request from the drone."""
    if os.path.exists(self.TOC_CACHE_FILE):
      with open(self.TOC_CACHE_FILE, "r") as f:
        self.toc = json.load(f)
      logger.info("Loaded TOC from cache.")
      self.ready.set()
      return

    # Step 1: Request TOC size
    logger.info("Requesting TOC size from drone...")
    self.udp_connection.send_packet(struct.pack("<B", self.CMD_TOC_INFO))

    data = self.udp_connection.receive_packet()
    if not data:
      logger.error("No response for TOC size request!")
      return

    # Step 2: Parse TOC size
    self.toc_size = struct.unpack("<B", data)[0]  # Assuming TOC size is a single byte
    logger.info(f"TOC contains {self.toc_size} variables.")

    # Step 3: Request each TOC entry
    for i in range(self.toc_size):
      self.request_toc_entry(i)

    # Save TOC to cache
    os.makedirs(os.path.dirname(self.TOC_CACHE_FILE), exist_ok=True)
    with open(self.TOC_CACHE_FILE, "w") as f:
      json.dump(self.toc, f)

    self.ready.set()
    logger.info("TOC fetching complete.")

  def request_toc_entry(self, index):
    """Request a specific TOC entry from the drone."""
    self.udp_connection.send_packet(struct.pack("<BB", self.CMD_TOC_ELEMENT, index))
    data = self.udp_connection.receive_packet()

    if not data:
      logger.error(f"TOC entry {index} not received!")
      return

    # Step 4: Parse TOC entry
    try:
      ident, group_len, name_len = struct.unpack_from("<BHH", data, 0)
      offset = 5
      group = data[offset:offset + group_len].decode().strip("\x00")
      offset += group_len
      name = data[offset:offset + name_len].decode().strip("\x00")
      offset += name_len

      self.toc[f"{group}.{name}"] = ident
      logger.info(f"Received TOC entry: {group}.{name} -> ID {ident}")

    except struct.error as e:
      logger.error(f"Failed to parse TOC entry {index}: {e}")

  def wait_for_toc(self, timeout=5):
    """Wait for TOC to be fully received before proceeding."""
    if not self.ready.wait(timeout):
      raise TimeoutError("TOC download timed out.")
