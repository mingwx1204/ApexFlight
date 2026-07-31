# -*- coding: utf-8 -*-
"""ApexFlight 打包脚本（v0.9）：PyInstaller 单文件 exe

用法：python build_exe.py
产物：dist/ApexFlight.exe（单文件，免安装）

打包内容：
- 主程序与全部模块（src/）
- assets/icon.png（窗口图标）+ 自动生成的 icon.ico（exe 图标）
- tools/blackbox_decode.exe 及其依赖 DLL（黑匣子解码器）
不打包：OllamaSetup.exe（体积太大，AI 功能由用户自行安装 Ollama）
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def make_ico() -> Path:
    """用 Pillow 把 assets/icon.png 转成多尺寸 icon.ico（exe 图标用）"""
    ico_path = ROOT / "assets" / "icon.ico"
    try:
        from PIL import Image
        img = Image.open(ROOT / "assets" / "icon.png")
        img.save(ico_path, sizes=[(16, 16), (32, 32), (48, 48),
                                  (64, 64), (128, 128), (256, 256)])
        return ico_path
    except Exception as e:
        print(f"⚠ 生成 .ico 失败（{e}），exe 将使用默认图标")
        return None


def main():
    import PyInstaller.__main__

    args = [
        str(ROOT / "src" / "main.py"),
        "--name=ApexFlight",
        "--onefile",                      # 单文件
        "--windowed",                     # 无控制台黑窗
        "--noconfirm",
        "--clean",
        f"--paths={ROOT / 'src'}",
        f"--distpath={ROOT / 'dist'}",
        f"--workpath={ROOT / 'build'}",
        f"--specpath={ROOT}",
        "--add-data", f"{ROOT / 'assets' / 'icon.png'};assets",
        "--add-data", f"{ROOT / 'assets' / 'qq_group_qr.png'};assets",
        # 隐藏导入：PyQt6/matplotlib 的钩子一般自动处理，这里兜底
        "--hidden-import=serial.tools.list_ports",
        "--hidden-import=matplotlib.backends.backend_qtagg",
    ]

    ico = make_ico()
    if ico:
        args.append(f"--icon={ico}")

    # 黑匣子解码器 + 全部依赖 DLL → 打包到 exe 内 tools/ 目录
    for dll in sorted((ROOT / "tools").glob("*.dll")):
        args += ["--add-binary", f"{dll};tools"]
    args += ["--add-binary", f"{ROOT / 'tools' / 'blackbox_decode.exe'};tools"]

    print("打包参数：", " ".join(str(a) for a in args), "\n")
    PyInstaller.__main__.run([str(a) for a in args])

    exe = ROOT / "dist" / "ApexFlight.exe"
    if exe.exists():
        size_mb = exe.stat().st_size / 1024 / 1024
        print(f"\n✅ 打包成功：{exe}（{size_mb:.1f} MB）")
    else:
        print("\n❌ 打包失败：未生成 dist/ApexFlight.exe")
        sys.exit(1)


if __name__ == "__main__":
    main()
