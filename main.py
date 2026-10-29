"""从源码目录启动 Zhiban Duck。

继续导出应用工厂和参数解析函数，兼容通过原入口模块调用的脚本。
"""

from zhiban_runtime.application import create_app
from zhiban_runtime.cli import main, parse_args

if __name__ == "__main__":
    main()
