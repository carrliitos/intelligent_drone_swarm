import os, os.path

def get_context(current_file_path):
  current_dir = os.path.abspath(os.path.join(current_file_path, os.pardir))
  parent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))

  return parent_dir
