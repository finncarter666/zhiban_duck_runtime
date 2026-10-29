"""以源码所在位置定位资源，避免依赖进程的工作目录。"""

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODEL_DIR = PROJECT_ROOT / "models"
WEB_DIR = PROJECT_ROOT / "assets" / "web"
