"""기존 실행 명령과 YAML 설정 선택을 제공하는 애플리케이션 진입점.

실제 구현은 :mod:`ffw_sh5_grasp.application.teleop`에 있다.
"""

import argparse
import os
import sys


def _select_config_before_import(argv):
    """설정 의존 모듈을 불러오기 전에 ``--config``를 환경 변수로 전달한다."""
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config")
    args, _unknown = parser.parse_known_args(argv)
    if args.config:
        os.environ["FFW_SH5_CONFIG"] = args.config


_select_config_before_import(sys.argv[1:])

from ffw_sh5_grasp.application.teleop import *  # noqa: E402,F401,F403

if __name__ == "__main__":
    main()
