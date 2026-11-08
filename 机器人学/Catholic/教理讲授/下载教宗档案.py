#!/usr/bin/env python3
"""Download post-Vatican-II papal archive documents from vatican.va."""

from __future__ import annotations

import argparse
import concurrent.futures
import html
import re
import sys
import subprocess
import time
import urllib.error
import urllib.parse
from collections import deque
from html.parser import HTMLParser
from pathlib import Path

BASE = "https://www.vatican.va"
ROOT = Path(__file__).resolve().parent / "教宗文献下载"
POPES = [
    ("leo-xiv", "良十四世"),
    ("francesco", "方济各"),
    ("benedict-xvi", "本笃十六世"),
    ("john-paul-ii", "若望保禄二世"),
    ("john-paul-i", "若望保禄一世"),
    ("paul-vi", "保禄六世"),
]
EXCLUDED_CATEGORIES = {"biography", "biografia"}
MONTHS = {
    month.lower(): i
    for i, month in enumerate(
        (
            "January February March April May June July August September October "
            "November December"
        ).split(),
        1,
    )
}


class PageParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.links: list[tuple[str, str]] = []
        self.title_parts: list[str] = []
        self.text_parts: list[str] = []
        self._anchor: list[str] | None = None
        self._in_title = False

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        attrs_dict = dict(attrs)
        if tag == "a" and attrs_dict.get("href"):
            self._anchor = [attrs_dict["href"] or "", ""]
        elif tag == "title":
            self._in_title = True

    def handle_data(self, data: str) -> None:
        self.text_parts.append(data)
        if self._anchor is not None:
            self._anchor[1] += data
        if self._in_title:
            self.title_parts.append(data)

    def handle_endtag(self, tag: str) -> None:
        if tag == "a" and self._anchor is not None:
            self.links.append((self._anchor[0], self._anchor[1].strip()))
            self._anchor = None
        elif tag == "title":
            self._in_title = False


class ReadableTextParser(HTMLParser):
    BLOCK_TAGS = {
        "address", "article", "blockquote", "br", "dd", "div", "dl", "dt",
        "h1", "h2", "h3", "h4", "h5", "h6", "li", "main", "ol", "p",
        "section", "table", "td", "th", "tr", "ul",
    }
    SKIP_TAGS = {
        "button", "footer", "form", "head", "header", "nav", "script",
        "style", "svg",
    }

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.parts: list[str] = []
        self.stack: list[tuple[str, bool]] = []
        self.main_depth = 0
        self.body_depth = 0
        self.found_main = False

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        attrs_dict = dict(attrs)
        classes = (attrs_dict.get("class") or "").lower()
        parent_skipped = self.stack[-1][1] if self.stack else False
        skipped = (
            parent_skipped
            or tag in self.SKIP_TAGS
            or "zoom-text" in classes
            or "textsize-list" in classes
            or "breadcrumb" in classes
        )
        self.stack.append((tag, skipped))
        if tag == "body":
            self.body_depth += 1
        if tag == "main":
            self.main_depth += 1
            self.found_main = True
        if tag in self.BLOCK_TAGS and not skipped and self._should_capture():
            self.parts.append("\n")

    def handle_endtag(self, tag: str) -> None:
        for index in range(len(self.stack) - 1, -1, -1):
            opened_tag, _ = self.stack[index]
            if opened_tag == tag:
                removed = self.stack[index:]
                del self.stack[index:]
                for removed_tag, _ in removed:
                    if removed_tag == "main":
                        self.main_depth = max(0, self.main_depth - 1)
                    elif removed_tag == "body":
                        self.body_depth = max(0, self.body_depth - 1)
                break
        if tag in self.BLOCK_TAGS and self._should_capture():
            self.parts.append("\n")

    def handle_data(self, data: str) -> None:
        skipped = self.stack[-1][1] if self.stack else False
        if not skipped and self._should_capture():
            self.parts.append(data)

    def _should_capture(self) -> bool:
        return self.main_depth > 0 or (not self.found_main and self.body_depth > 0)

    def text(self) -> str:
        lines = []
        for line in "".join(self.parts).splitlines():
            cleaned = re.sub(r"\s+", " ", line).strip()
            if cleaned and (not lines or lines[-1] != cleaned):
                lines.append(cleaned)
        return "\n\n".join(lines) + ("\n" if lines else "")


def readable_text(raw_html: bytes) -> str:
    parser = ReadableTextParser()
    parser.feed(raw_html.decode("utf-8", "replace"))
    return parser.text()


def fetch(url: str) -> bytes:
    result = subprocess.run(
        [
            "curl",
            "--fail",
            "--silent",
            "--show-error",
            "--location",
            "--http1.1",
            "--retry",
            "2",
            "--retry-connrefused",
            "--retry-delay",
            "1",
            "--max-time",
            "60",
            "--user-agent",
            "PapalArchiveDownloader/1.0 (personal research; respectful rate)",
            url,
        ],
        check=False,
        capture_output=True,
    )
    if result.returncode:
        message = result.stderr.decode("utf-8", "replace").strip()
        raise OSError(f"curl exited {result.returncode}: {message}")
    return result.stdout


def normalized_url(href: str, page_url: str) -> str | None:
    url = urllib.parse.urljoin(page_url, href)
    parsed = urllib.parse.urlparse(url)
    if parsed.scheme not in {"http", "https"} or parsed.netloc not in {
        "www.vatican.va",
        "vatican.va",
    }:
        return None
    return urllib.parse.urlunparse(parsed._replace(fragment=""))


def excluded_index(path: str, slug: str) -> bool:
    relative = path.split(f"/content/{slug}/en/", 1)[-1]
    category = relative.split("/", 1)[0].removesuffix(".index.html").lower()
    return category in EXCLUDED_CATEGORIES


def get_index_documents(slug: str) -> list[tuple[str, str]]:
    root_url = f"{BASE}/content/{slug}/en.html"
    root_parser = PageParser()
    root_parser.feed(fetch(root_url).decode("utf-8", "replace"))
    prefix = f"/content/{slug}/en/"
    pending: deque[str] = deque()
    visited: set[str] = set()
    documents: set[str] = set()

    for href, _ in root_parser.links:
        url = normalized_url(href, root_url)
        if not url:
            continue
        path = urllib.parse.urlparse(url).path
        if path.startswith(prefix) and path.endswith(".index.html"):
            if not excluded_index(path, slug):
                pending.append(url)

    def parse_index(index_url: str) -> tuple[str, PageParser | None]:
        try:
            parser = PageParser()
            parser.feed(fetch(index_url).decode("utf-8", "replace"))
            return index_url, parser
        except (OSError, TimeoutError, UnicodeError) as error:
            print(f"目录读取失败：{index_url}：{error}", file=sys.stderr)
            return index_url, None

    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as pool:
        while pending:
            batch = []
            while pending:
                url = pending.popleft()
                if url not in visited:
                    visited.add(url)
                    batch.append(url)
            for index_url, parser in pool.map(parse_index, batch):
                if parser is None:
                    continue
                for href, _ in parser.links:
                    url = normalized_url(href, index_url)
                    if not url:
                        continue
                    path = urllib.parse.urlparse(url).path
                    if not path.startswith(prefix):
                        continue
                    if path.endswith(".index.html"):
                        if not excluded_index(path, slug) and url not in visited:
                            pending.append(url)
                    elif path.lower().endswith(".html") and path != prefix.rstrip("/"):
                        documents.add(url)

    return [(slug, url) for url in sorted(documents)]


def route_for_language(url: str, language: str) -> str:
    if language == "zh_cn":
        return url.replace("/en/", "/zh_cn/", 1)
    return url


def parse_date(url: str, text: str, title: str) -> tuple[str | None, str | None]:
    url_path = urllib.parse.urlparse(url).path
    basename = url_path.rsplit("/", 1)[-1]
    match = re.search(r"(?<!\d)((?:19|20)\d{6})(?!\d)", basename)
    if match:
        value = match.group(1)
        return f"{value[:4]}-{value[4:6]}-{value[6:8]}", value[:4]
    match = re.search(r"/((?:19|20)\d{2})/", url_path)
    if match:
        return f"{match.group(1)}-00-00", match.group(1)

    combined = f"{title} {text[:6000]}"
    match = re.search(r"\b((?:19|20)\d{2})[-/.年 ](\d{1,2})[-/.月 ](\d{1,2})日?\b", combined)
    if match:
        year, month, day = map(int, match.groups())
        if 1 <= month <= 12 and 1 <= day <= 31:
            return f"{year:04d}-{month:02d}-{day:02d}", str(year)

    match = re.search(
        r"\b(\d{1,2})\s+(January|February|March|April|May|June|July|August|"
        r"September|October|November|December)\s+((?:19|20)\d{2})\b",
        combined,
        re.IGNORECASE,
    )
    if match:
        day, month, year = match.groups()
        return f"{int(year):04d}-{MONTHS[month.lower()]:02d}-{int(day):02d}", year
    match = re.search(
        r"\b(January|February|March|April|May|June|July|August|September|October|"
        r"November|December)\s+(\d{1,2}),?\s+((?:19|20)\d{2})\b",
        combined,
        re.IGNORECASE,
    )
    if match:
        month, day, year = match.groups()
        return f"{int(year):04d}-{MONTHS[month.lower()]:02d}-{int(day):02d}", year
    match = re.search(r"\b((?:19|20)\d{2})\b", basename)
    if match:
        return f"{match.group(1)}-00-00", match.group(1)
    match = re.search(r"\b((?:19|20)\d{2})\b", combined)
    if match:
        return f"{match.group(1)}-00-00", match.group(1)
    return None, None


def safe_name(value: str, limit: int = 120) -> str:
    value = html.unescape(re.sub(r"<[^>]*>", "", value))
    value = re.sub(r'[<>:"/\\|?*\x00-\x1f]', "", value)
    value = re.sub(r"\s+", " ", value).strip(" .-_")
    return value[:limit].strip(" .-_") or "未命名"


def category_for(url: str) -> str:
    relative = urllib.parse.urlparse(url).path.split("/en/", 1)[-1]
    parts = [part for part in relative.split("/") if part and not part.endswith(".html")]
    if not parts:
        return "其他"
    return safe_name(parts[0], 80)


def is_pdf(url: str) -> bool:
    return urllib.parse.urlparse(url).path.lower().endswith(".pdf")


def write_resource(url: str, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.stat().st_size:
        return
    data = fetch(url)
    temp_path = path.with_suffix(path.suffix + ".part")
    temp_path.write_bytes(data)
    temp_path.replace(path)


def download_document(
    slug: str, pope_name: str, english_url: str, delay: float
) -> tuple[int, list[str]]:
    errors: list[str] = []
    selected_url = route_for_language(english_url, "zh_cn")
    language = "zh_cn"
    try:
        raw = fetch(selected_url)
    except (urllib.error.URLError, TimeoutError, OSError):
        selected_url = english_url
        language = "en"
        try:
            raw = fetch(selected_url)
        except (OSError, TimeoutError) as error:
            return 0, [f"{english_url}: {error}"]

    page_html = raw.decode("utf-8", "replace")
    parser = PageParser()
    parser.feed(page_html)
    title = "".join(parser.title_parts).strip() or urllib.parse.urlparse(english_url).path.rsplit("/", 1)[-1]
    plain_text = " ".join(parser.text_parts)
    date, year = parse_date(english_url, plain_text, title)
    if slug == "paul-vi" and date and date < "1965-12-08":
        return 0, []

    category = category_for(english_url)
    file_stem = safe_name(f"{date or '日期待核'}_{title}")
    directory = ROOT / pope_name / category
    html_path = directory / f"{file_stem}_{language}.html"
    try:
        html_path.parent.mkdir(parents=True, exist_ok=True)
        if not html_path.exists() or not html_path.stat().st_size:
            temp_path = html_path.with_suffix(html_path.suffix + ".part")
            temp_path.write_bytes(raw)
            temp_path.replace(html_path)
        text_path = html_path.with_suffix(".txt")
        if not text_path.exists() or not text_path.stat().st_size:
            text_path.write_text(readable_text(raw), encoding="utf-8")
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        errors.append(f"{selected_url}: {error}")
        return 0, errors

    saved = 1
    pdf_urls: list[str] = []
    for href, _ in parser.links:
        url = normalized_url(href, selected_url)
        if url and is_pdf(url) and url not in pdf_urls:
            pdf_urls.append(url)
    for index, pdf_url in enumerate(pdf_urls, 1):
        original_basename = urllib.parse.unquote(
            urllib.parse.urlparse(pdf_url).path.rsplit("/", 1)[-1]
        )
        suffix = Path(original_basename).suffix or ".pdf"
        pdf_stem = file_stem if len(pdf_urls) == 1 else f"{file_stem}_{index}"
        pdf_path = directory / f"{pdf_stem}_{language}{suffix}"
        try:
            time.sleep(delay)
            write_resource(pdf_url, pdf_path)
            saved += 1
        except (urllib.error.URLError, TimeoutError, OSError) as error:
            errors.append(f"{pdf_url}: {error}")
    time.sleep(delay)
    return saved, errors


def main() -> int:
    argument_parser = argparse.ArgumentParser(description=__doc__)
    argument_parser.add_argument(
        "--dry-run", action="store_true", help="只扫描目录并统计，不下载文件"
    )
    argument_parser.add_argument(
        "--convert-existing",
        action="store_true",
        help="把已下载网页转换成可直接阅读的纯文本，然后退出",
    )
    argument_parser.add_argument(
        "--workers", type=int, default=8, help="并发下载数，默认 8"
    )
    argument_parser.add_argument(
        "--delay", type=float, default=0.1, help="单条目间隔秒数，默认 0.1"
    )
    argument_parser.add_argument(
        "--popes",
        nargs="+",
        choices=[slug for slug, _ in POPES],
        default=[slug for slug, _ in POPES],
        help="只下载指定教宗（默认包含全部教宗）",
    )
    args = argument_parser.parse_args()
    if args.workers < 1:
        argument_parser.error("--workers 必须大于 0")
    if args.convert_existing:
        converted = 0
        for html_path in ROOT.rglob("*.html"):
            text_path = html_path.with_suffix(".txt")
            if text_path.exists() and text_path.stat().st_size:
                continue
            text_path.write_text(
                readable_text(html_path.read_bytes()), encoding="utf-8"
            )
            converted += 1
        print(f"已生成 {converted} 个纯文本文件。", flush=True)
        return 0

    selected_popes = set(args.popes)
    failures: list[str] = []
    total_saved = 0
    total_records = 0
    download_priority = {
        "john-paul-i": 0,
        "paul-vi": 1,
        "benedict-xvi": 2,
        "francesco": 3,
        "john-paul-ii": 4,
        "leo-xiv": 5,
    }
    for slug, pope_name in sorted(POPES, key=lambda pope: download_priority[pope[0]]):
        if slug not in selected_popes:
            continue
        print(f"扫描 {pope_name} ({slug}) …", flush=True)
        try:
            documents = get_index_documents(slug)
        except (urllib.error.URLError, TimeoutError, OSError) as error:
            message = f"{pope_name}主页读取失败：{slug}: {error}"
            print(message, file=sys.stderr, flush=True)
            failures.append(message)
            continue
        total_records += len(documents)
        print(f"  找到 {len(documents)} 个网页入口", flush=True)

        if args.dry_run:
            continue

        ROOT.mkdir(parents=True, exist_ok=True)
        completed = 0
        pope_saved = 0
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
            future_records = {
                pool.submit(download_document, slug, pope_name, url, args.delay): url
                for _, url in documents
            }
            for future in concurrent.futures.as_completed(future_records):
                url = future_records[future]
                try:
                    count, errors = future.result()
                    pope_saved += count
                    total_saved += count
                    failures.extend(errors)
                except Exception as error:
                    failures.append(f"{url}: {type(error).__name__}: {error}")
                completed += 1
                if completed % 100 == 0 or completed == len(future_records):
                    print(
                        f"{pope_name}进度 {completed}/{len(future_records)}，"
                        f"本任期保存 {pope_saved} 个文件；总计 {total_saved}，"
                        f"失败 {len(failures)} 条",
                        flush=True,
                    )

    if args.dry_run:
        print(f"合计网页入口：{total_records}")
        return 0

    if failures:
        error_file = ROOT / "下载失败记录.txt"
        error_file.write_text("\n".join(failures) + "\n", encoding="utf-8")
        print(f"失败记录已写入：{error_file}", file=sys.stderr)
    print(
        f"完成：扫描到 {total_records} 个页面入口，保存 {total_saved} 个文件；"
        f"失败 {len(failures)} 条。目录：{ROOT}",
        flush=True,
    )
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
