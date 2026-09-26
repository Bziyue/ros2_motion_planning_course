#!/usr/bin/env python3
"""Render a PDF for visual review; successful rendering does not certify layout.

Usage: python3 scripts/render_pdf_review.py PDF OUTPUT_DIR [--latex-log LOG]
Dependencies: Poppler (pdfinfo/pdftoppm) and Pillow (host python3-pil).
The output directory is generated scratch space, separate from PDF sources.
"""
import argparse
from datetime import datetime
import hashlib
import html
from pathlib import Path
import re
import subprocess

from PIL import Image, ImageDraw, ImageFont


def make_contact_sheets(pages, output):
    """Create four-page overviews without changing page aspect ratios."""
    font = ImageFont.truetype("DejaVuSans.ttf", 18)
    for start in range(0, len(pages), 4):
        sheet = Image.new("RGB", (1280, 1840), "#e5eaf0")
        draw = ImageDraw.Draw(sheet)
        for offset, path in enumerate(pages[start:start + 4]):
            x = 20 + (offset % 2) * 620
            y = 20 + (offset // 2) * 900
            draw.text((x, y), f"PDF page {start + offset + 1}",
                      font=font, fill="#183447")
            with Image.open(path) as page:
                page.thumbnail((600, 850), Image.Resampling.LANCZOS)
                sheet.paste(page, (x + (600 - page.width) // 2, y + 30))
        end = min(start + 4, len(pages))
        sheet.save(output / f"sheet-{start + 1:02d}-{end:02d}.png")


def main():
    """Render every physical page and reset the review checklist on each run."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pdf", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--latex-log", type=Path)
    args = parser.parse_args()
    pdf, output = args.pdf.resolve(), args.output.resolve()
    info = subprocess.run(["pdfinfo", str(pdf)], check=True,
                          capture_output=True, text=True).stdout
    page_count = int(re.search(r"^Pages:\s+(\d+)", info, re.MULTILINE).group(1))
    digest = hashlib.sha256(pdf.read_bytes()).hexdigest()
    warnings = []
    if args.latex_log:
        warnings = [
            line.strip()
            for line in args.latex_log.read_text(errors="replace").splitlines()
            if any(word in line for word in ("Overfull \\", "Underfull \\",
                                             "Missing character:"))
        ]

    pages_dir = output / "pages"
    pages_dir.mkdir(parents=True, exist_ok=True)
    # Remove only this tool's generated files so a shorter PDF leaves no old pages.
    for path in [*pages_dir.glob("page-*.png"), *output.glob("sheet-*.png"),
                 output / "index.html", output / "CHECKLIST.md"]:
        path.unlink(missing_ok=True)
    subprocess.run(["pdftoppm", "-r", "144", "-png", str(pdf),
                    str(pages_dir / "page")], check=True)
    pages = sorted(pages_dir.glob("page-*.png"))
    if len(pages) != page_count:
        raise RuntimeError(f"Expected {page_count} pages, rendered {len(pages)}")
    make_contact_sheets(pages, output)

    cards = "\n".join(
        f'<figure><a href="pages/{page.name}">'
        f'<img src="pages/{page.name}" loading="lazy" alt="PDF 第 {i} 页">'
        f'</a><figcaption>PDF 第 {i} 页</figcaption></figure>'
        for i, page in enumerate(pages, 1)
    )
    log_text = "\n".join(warnings) or "未发现 Overfull、Underfull 或缺字提示。"
    if args.latex_log is None:
        log_text = "未提供 LaTeX 日志；本次只准备图像，请另外检查构建日志。"
    (output / "index.html").write_text(f"""<!doctype html>
<html lang="zh-CN"><meta charset="utf-8">
<title>PDF 视觉复核：{html.escape(pdf.name)}</title>
<style>
body {{margin:24px; color:#183447; background:#e5eaf0; font-family:sans-serif}}
main {{display:grid; grid-template-columns:repeat(auto-fit,minmax(360px,1fr)); gap:24px}}
figure {{margin:0}} img {{width:100%; height:auto; background:white}}
figcaption {{padding:8px 0}} code,pre {{overflow-wrap:anywhere; white-space:pre-wrap}}
</style>
<h1>待视觉复核：{html.escape(pdf.name)}</h1>
<p>{page_count} 个物理页，144 dpi。点击页面可放大；页号包含封面和目录。</p>
<p>渲染完成不代表视觉检查通过。逐页查看，并放大核对图形、标注、公式和代码。</p>
<p>PDF SHA-256：<code>{digest}</code></p>
<details><summary>LaTeX 排版提示</summary><pre>{html.escape(log_text)}</pre></details>
<main>{cards}</main></html>
""", encoding="utf-8")
    (output / "CHECKLIST.md").write_text(f"""# 待完成的 PDF 视觉复核

- PDF：{pdf}
- SHA-256：{digest}
- 生成时间：{datetime.now().astimezone().isoformat(timespec="seconds")}
- 物理页数：{page_count}（包括封面、目录），分辨率：144 dpi

每次渲染重置本清单；工具不能判断线条重叠、几何失真或标注含义。

- [ ] 查看全部页面或联系表，核对分页、边距和页眉页脚。
- [ ] 按阅读大小或放大检查图表、公式、表格、代码；联系表不能替代细节检查。
- [ ] 核对标注与连线、圆与网格的等比例、截图纵横比、箭头遮挡和字体。
- [ ] 若发现问题，修改源文件，重新构建和渲染，检查受影响页面及相邻页面。
- [ ] 在 docs/VALIDATION.md 记录本次 PDF 哈希、范围、问题与复核结果，再提交。

## LaTeX 排版提示

{log_text}
""", encoding="utf-8")
    print(f"已渲染 {page_count} 页（144 dpi），生成四页联系表和逐页浏览入口。")
    print(f"待视觉复核：{output / 'index.html'}")
    print(f"待填写清单：{output / 'CHECKLIST.md'}")
    # Still render when TeX reports overflow so the offending page can be inspected.
    if any(line.startswith(("Overfull", "Missing character:")) for line in warnings):
        print("发现溢出或缺字提示，请修复后重新构建。")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
