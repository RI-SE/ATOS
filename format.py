#!/usr/bin/python3
#
# To enable formatting in VS Code install the following extensions:
# $ code --install-extension charliermarsh.ruff && code --install-extension xaver.clang-format
# Add the following to ~/.config/Code/User/settings.json
#
# "[python]": {
#     "editor.defaultFormatter": "charliermarsh.ruff",
#     "editor.codeActionsOnSave": {
#         "source.fixAll": "always",
#     }
# },
# "ruff.path": [
#     "/home/<user>/.cache/ruff-x86_64-unknown-linux-gnu/ruff"
# ],
# "ruff.lint.extendSelect": [
#     "I"
# ],
# "editor.formatOnSave": true,
# "clang-format.executable": "/home/<user>/.cache/clang-format",
# "files.insertFinalNewline": true,
# "files.trimFinalNewlines": true,
# "files.trimTrailingWhitespace": true,
#
# If you have problems:
# 1. make sure you have no other extensions trying to format cpp files.
# 2. you have selected a Python interpreter.

import argparse
import hashlib
import json
import os
import stat
import subprocess
import sys
import tarfile
from pathlib import Path

import requests


def sha256sum(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def download_if_incorrect_hash(path, url, sha256):
    updated_file = False
    path.parent.mkdir(parents=True, exist_ok=True)
    if (not path.is_file()) or (sha256sum(path) != sha256):
        r = requests.get(url, allow_redirects=True)
        with open(path, "wb") as f:
            f.write(r.content)
            updated_file = True
    assert sha256sum(path) == sha256
    return updated_file


def download_and_unpack_if_incorrect_hash(path, url, sha256):
    if download_if_incorrect_hash(path, url, sha256):
        with tarfile.open(path, "r:gz") as f:
            f.extractall(path=path.parent)


def make_file_runnable(path):
    st = os.stat(path)
    os.chmod(path, st.st_mode | stat.S_IEXEC)


def is_git_index_dirty():
    p = subprocess.run(
        ["git", "status", "-s"],
        text=True,
        stdout=subprocess.PIPE,
        check=True,
    )
    files = [line for line in p.stdout.splitlines() if "M " in line or "A " in line]
    return len(files)


def git_touched_files():
    p = subprocess.run(
        ["git", "merge-base", "HEAD", "origin/dev"],
        text=True,
        check=True,
        stdout=subprocess.PIPE,
    )
    d = subprocess.run(
        ["git", "diff", "--name-only", f"{p.stdout.strip()}..HEAD"],
        text=True,
        check=True,
        stdout=subprocess.PIPE,
    )
    return d.stdout.splitlines()


def git_ls_files():
    p = subprocess.run(
        ["git", "ls-files"],
        text=True,
        check=True,
        stdout=subprocess.PIPE,
    )

    return p.stdout.splitlines()


def cache_folder():
    return Path("~/.cache/rise").expanduser()


def clang_format(files, check):
    cpp_files = [f for f in files if "." in f and f.rsplit(".")[1] in ["hpp", "cpp"]]
    if len(cpp_files) == 0:
        return []

    path = cache_folder() / "clang-format"
    url = "https://github.com/cpp-linter/clang-tools-static-binaries/releases/download/master-67c95218/clang-format-19_linux-amd64"
    sha256 = "6ede4977469da4325bb7109916e41c067f9e01fa3bddd3c90090c8609d8e364e"

    download_if_incorrect_hash(path, url, sha256)
    make_file_runnable(path)

    problematic_files = []
    p = subprocess.run(
        [path, "--Werror", "--dry-run", *cpp_files],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    problematic_files = list(
        set(
            line.split(":")[0]
            for line in p.stderr.splitlines()
            if "-Wclang-format-violations" in line
        )
    )

    if len(problematic_files) > 0 and not check:
        p = subprocess.run(
            [path, "-i", "--verbose", *problematic_files],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    return problematic_files


def ruff(files, check):
    files = [f for f in files if "." in f and f.rsplit(".")[1] == "py"]
    if len(files) == 0:
        return []

    tar_path = cache_folder() / "ruff-x86_64-unknown-linux-gnu.tar.gz"
    url = "https://github.com/astral-sh/ruff/releases/download/0.9.6/ruff-x86_64-unknown-linux-gnu.tar.gz"
    sha256 = "bed850f15d4d5aaaef2b6a131bfecd5b9d7d3191596249d07e576bd9fd37078e"
    download_and_unpack_if_incorrect_hash(tar_path, url, sha256)
    ruff_binary = cache_folder() / "ruff-x86_64-unknown-linux-gnu/ruff"
    make_file_runnable(ruff_binary)

    format_cmd = [ruff_binary, "format"]
    p = subprocess.run(
        [*format_cmd, "--check", *files],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    unformatted_files = [
        line.replace("Would reformat: ", "")
        for line in p.stdout.splitlines()
        if "Would reformat" in line
    ]
    if len(unformatted_files) > 0 and not check:
        p = subprocess.run(
            [*format_cmd, *unformatted_files],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    lint_cmd = [ruff_binary, "check", "--extend-select=I"]
    p = subprocess.run(
        [*lint_cmd, "--output-format=json", *files],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    problematic_files = [
        str(Path(f["filename"]).relative_to(Path.cwd())) for f in json.loads(p.stdout)
    ]

    if len(problematic_files) == 0:
        return unformatted_files

    if not check:
        lint_cmd.append("--fix")
    subprocess.run([*lint_cmd, *files], text=True)

    return list(set(problematic_files) | set(unformatted_files))


def filter_files_to_ignore(files):
    ignore = ("atos/modules/ObjectControl/inc/sml.hpp",)
    return [f for f in files if f not in ignore]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="Don't apply any changes.")
    parser.add_argument("--all-files", action="store_true", help="Format all files.")

    args = parser.parse_args()

    if is_git_index_dirty():
        print(
            "git index contain unstaged files, add and commit your changes before attempting linting."
        )
        return 1

    if args.all_files:
        files = filter_files_to_ignore(git_ls_files())
    else:
        files = filter_files_to_ignore(git_touched_files())

    problematic_files = []
    problematic_files.extend(ruff(files, args.check))
    problematic_files.extend(clang_format(files, args.check))

    if len(problematic_files) > 0:
        if args.check:
            print(
                "\n\nThe following files needs formatting and/or contains problems:\n",
                "\n ".join(sorted(problematic_files)),
            )
        else:
            print(
                "\n\nThe following files were formated and/or contains problems:\n",
                "\n ".join(sorted(problematic_files)),
            )
        return 1
    else:
        return 0


if __name__ == "__main__":
    sys.exit(main())
