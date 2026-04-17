#!/usr/bin/env python3
import argparse
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import List, Optional, Tuple
from urllib import request


DefaultBuildCommand = ["build.bat"]
ProxyEnvKeys = ["HTTP_PROXY", "HTTPS_PROXY", "http_proxy", "https_proxy"]


def ParseArgs() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="自动执行 build 脚本，遇到下载失败时自动补下载到 vcpkg/downloads 后重试。"
    )
    parser.add_argument(
        "--build-cmd",
        default="build.bat",
        help='构建命令，例如 "build.bat" 或 "build_cn.bat"',
    )
    parser.add_argument(
        "--max-retries",
        type=int,
        default=30,
        help="最大自动修复重试次数，默认 30",
    )
    parser.add_argument(
        "--vcpkg-root",
        default="",
        help="可选，显式指定 vcpkg 根目录",
    )
    parser.add_argument(
        "--proxy",
        default="",
        help='可选，显式代理，例如 "http://127.0.0.1:21081"',
    )
    return parser.parse_args()


def GetProxyFromEnv() -> str:
    for key in ProxyEnvKeys:
        value = os.environ.get(key, "").strip()
        if value:
            return value
    return ""


def NormalizeBuildCommand(build_cmd: str) -> List[str]:
    if build_cmd.lower().endswith(".bat"):
        if not build_cmd.startswith(".\\"):
            return [".\\" + build_cmd]
        return [build_cmd]
    return build_cmd.split()


def RunBuildCommand(command: List[str], env: dict) -> Tuple[int, str]:
    print(f"[INFO] Run command: {' '.join(command)}")
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        encoding="utf-8",
        errors="replace",
        shell=False,
        env=env,
    )
    output_lines: List[str] = []
    assert process.stdout is not None
    for line in process.stdout:
        print(line, end="")
        output_lines.append(line)
    process.wait()
    return process.returncode, "".join(output_lines)


def DetectVcpkgRoot(build_output: str, explicit_vcpkg_root: str) -> Path:
    if explicit_vcpkg_root:
        return Path(explicit_vcpkg_root).expanduser().resolve()
    match = re.search(r'\[INFO\] Use vcpkg root:\s*"([^"]+)"', build_output)
    if match:
        return Path(match.group(1)).expanduser().resolve()
    fallback = Path(os.path.expandvars(r"%USERPROFILE%\vcpkg")).expanduser().resolve()
    return fallback


def ParseFailedDownloadInfo(build_output: str) -> Optional[Tuple[str, List[str]]]:
    filename = ""
    download_line_match_arrow = re.findall(
        r"Downloading\s+(https?://\S+)\s*->\s*([^\s]+)",
        build_output,
    )
    filename_match = re.findall(r"Trying to download\s+([^\s]+)\s+using asset cache", build_output)
    if filename_match:
        filename = filename_match[-1].strip()

    download_line_match = re.findall(
        r"Downloading\s+([^\s,]+),\s*trying\s+(https?://\S+)",
        build_output,
    )
    if download_line_match_arrow and not filename:
        filename = download_line_match_arrow[-1][1].strip()
    if download_line_match and not filename:
        filename = download_line_match[-1][0].strip()

    urls: List[str] = []
    urls.extend([item[0].strip() for item in download_line_match_arrow])
    urls.extend([item[1].strip() for item in download_line_match])

    authoritative = re.findall(r"Asset cache miss; trying authoritative source\s+(https?://\S+)", build_output)
    urls.extend(authoritative)

    trying_urls = re.findall(r"Trying\s+(https?://\S+)", build_output)
    urls.extend(trying_urls)

    dedup_urls: List[str] = []
    for url in urls:
        url = url.strip().rstrip(".,")
        if url and url not in dedup_urls:
            dedup_urls.append(url)
    if not filename and dedup_urls:
        filename = Path(dedup_urls[-1]).name
    if not filename:
        return None
    if not dedup_urls:
        return None
    return filename, dedup_urls


def DownloadFile(urls: List[str], destination: Path, proxy: str) -> bool:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() and destination.stat().st_size > 0:
        print(f"[INFO] Local file already exists: {destination}")
        return True

    handlers = []
    if proxy:
        handlers.append(request.ProxyHandler({"http": proxy, "https": proxy}))
    opener = request.build_opener(*handlers)

    for url in urls:
        try:
            print(f"[INFO] Downloading: {url}")
            with opener.open(url, timeout=60) as response:
                data = response.read()
            if not data:
                print(f"[WARN] Empty response: {url}")
                continue
            destination.write_bytes(data)
            print(f"[INFO] Download completed: {destination}")
            return True
        except Exception as exc:
            print(f"[WARN] Download failed from {url}: {exc}")
    return False


def Main() -> int:
    args = ParseArgs()
    command = NormalizeBuildCommand(args.build_cmd)

    env = os.environ.copy()
    proxy = args.proxy.strip() or GetProxyFromEnv()
    if proxy:
        env["HTTP_PROXY"] = proxy
        env["HTTPS_PROXY"] = proxy
        env["http_proxy"] = proxy
        env["https_proxy"] = proxy
        print(f"[INFO] Use proxy: {proxy}")

    for round_index in range(1, args.max_retries + 1):
        print(f"\n[INFO] ===== Build round {round_index}/{args.max_retries} =====")
        code, output = RunBuildCommand(command, env)
        if code == 0:
            print("[INFO] Build completed successfully.")
            return 0

        failed = ParseFailedDownloadInfo(output)
        if not failed:
            print("[ERROR] Build failed, and no downloadable artifact was detected from logs.")
            return code

        filename, urls = failed
        vcpkg_root = DetectVcpkgRoot(output, args.vcpkg_root)
        download_path = vcpkg_root / "downloads" / filename
        print(f"[INFO] Detected failed download file: {filename}")
        print(f"[INFO] Target path: {download_path}")
        print("[INFO] Try manual download to local cache...")
        downloaded = DownloadFile(urls, download_path, proxy)
        if not downloaded:
            print("[ERROR] Auto download failed for all candidate URLs.")
            return 1
        print("[INFO] Cached file ready, retry build...")

    print(f"[ERROR] Reached max retries: {args.max_retries}")
    return 2


if __name__ == "__main__":
    sys.exit(Main())
