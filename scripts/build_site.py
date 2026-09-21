from __future__ import annotations

import html
import posixpath
import re
import shutil
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit

try:
    import markdown
except ImportError:
    raise SystemExit("Install site dependencies: python3 -m pip install -r scripts/requirements-site.txt")

ROOT = Path(__file__).resolve().parents[1]
SITE_SRC = ROOT / "site"
OUT = ROOT / "_site"
PAGES = [
    ("overview", "Overview", "CRISP capabilities, assembly scenarios, and guides."),
    ("installation", "Installation", "Requirements, build, run, and troubleshooting."),
    ("examples", "Examples", "Run the peg insertion and gear assembly scenarios and inspect contact."),
    ("model-construction", "Model construction", "Bodies, joints, collision geometry, and actuation."),
    ("contact-solvers", "Contact solvers", "CANAL, SubADMM, and numerical settings."),
    ("using-crisp", "Using CRISP", "Build, control, step, and inspect a simulation."),
    ("api-reference", "API reference", "Operations, state, ownership, and callbacks."),
    ("release-notes", "Release notes", "Feature updates and breaking changes by version."),
    ("publications", "Publications", "Method papers and citation information."),
    ("licenses", "Licenses", "Sources and license information for example assets."),
]



class References(HTMLParser):
    def __init__(self):
        super().__init__()
        self.urls = []
        self.ids = set()

    def handle_starttag(self, tag, attrs):
        for key, value in attrs:
            if not value:
                continue
            if key == "id":
                if value in self.ids:
                    raise ValueError(f"Duplicate HTML id: {value}")
                self.ids.add(value)
            if key in {"src", "href", "poster", "data-video", "data-clean-poster", "data-contact-poster"}:
                self.urls.append(value)


def render_header(prefix="", active="home"):
    links = [
        ("home", "Home", prefix or "./"),
        ("documentation", "Documentation", prefix + "docs/overview/"),
        ("publications", "Publications", prefix + "docs/publications/"),
        ("github", "GitHub ↗", "https://github.com/INRoL/crisp"),
    ]
    navigation = "".join(
        f'<a href="{href}"' + (' aria-current="page"' if name == active else '') + f'>{label}</a>'
        for name, label, href in links
    )
    return f'''<header class="header wrap">
<a class="wordmark" href="{prefix or './'}" aria-label="CRISP home"><img class="brand-symbol" src="{prefix}brand-icon.svg?v=gripper-symmetric" alt="" width="36" height="36">CRISP</a>
<nav aria-label="Main navigation">{navigation}</nav>
</header>'''


def render_doc(slug, title, description):
    source = ROOT / "docs" / f"{slug}.md"
    parser = markdown.Markdown(extensions=["fenced_code", "tables", "toc", "sane_lists"])
    content = parser.convert(source.read_text(encoding="utf-8"))

    if slug == "release-notes":
        versions = []

        def version_heading(match):
            version = match.group(1)
            version_id = "v-" + version.replace(".", "-")
            versions.append({"id": version_id, "label": version, "sections": []})
            return f'<h2 id="{version_id}">{version}</h2>'

        content = re.sub(
            r'<h2 id="[^"]+">(\d+\.\d+\.\d+)</h2>',
            version_heading,
            content,
        )
        version_by_id = {version["id"]: version for version in versions}
        current_version = None

        def scope_release_heading(match):
            nonlocal current_version
            level, heading_id, heading_html = match.groups()
            if level == "2":
                current_version = version_by_id.get(heading_id)
            elif current_version is not None:
                base_id = re.sub(r"_\d+$", "", heading_id)
                heading_id = f'{current_version["id"]}-{base_id}'
                heading_text = html.unescape(re.sub(r"<[^>]+>", "", heading_html))
                current_version["sections"].append((heading_id, heading_text))
            return f'<h{level} id="{heading_id}">{heading_html}</h{level}>'

        content = re.sub(
            r'<h([23]) id="([^"]+)">(.*?)</h\1>',
            scope_release_heading,
            content,
            flags=re.S,
        )

        latest_version = max(
            versions,
            key=lambda version: tuple(int(part) for part in version["label"].split(".")),
        )
        toc_items = []
        for version in versions:
            version_link = f'<a href="#{version["id"]}">{version["label"]}</a>'
            if version["sections"]:
                section_links = "".join(
                    f'<li><a href="#{section_id}">{html.escape(section_title)}</a></li>'
                    for section_id, section_title in version["sections"]
                )
                open_attribute = " open" if version is latest_version else ""
                toc_items.append(
                    '<li class="release-toc-version">'
                    f'<details{open_attribute}><summary>{version_link}</summary>'
                    f'<ul>{section_links}</ul></details></li>'
                )
            else:
                toc_items.append(f'<li class="release-toc-version">{version_link}</li>')
        toc = '<div class="toc"><ul>' + "".join(toc_items) + "</ul></div>"
    else:
        toc = parser.toc

    def rewrite(match):
        attribute, value = match.groups()
        url = urlsplit(html.unescape(value))
        if url.scheme or url.netloc or not url.path:
            return match.group(0)
        target = posixpath.normpath("docs/" + unquote(url.path))
        if target.startswith("docs/") and target.endswith(".md"):
            target_slug = Path(target).stem
            if target_slug not in {page[0] for page in PAGES}:
                raise ValueError(f"Unpublished document link: {target}")
            result = f"../{target_slug}/"
            if url.fragment:
                result += "#" + url.fragment
        else:
            if not (ROOT / target).is_file():
                raise ValueError(f"Missing repository link: {target}")
            result = "https://github.com/INRoL/crisp/blob/main/" + target
        return f'{attribute}="{html.escape(result, quote=True)}"'

    content = re.sub(r'(href|src)="([^"]+)"', rewrite, content)
    navigation = []
    groups = {"overview": "Get started", "model-construction": "Models and methods", "using-crisp": "C++ development", "release-notes": "Project"}
    for name, label, _ in PAGES:
        if name in groups:
            navigation.append(f'<p class="nav-group">{groups[name]}</p>')
        current = ' aria-current="page"' if name == slug else ""
        navigation.append(f'<a href="../{name}/"{current}>{label}</a>')
    nav = "".join(navigation)
    page_index = next(i for i, page in enumerate(PAGES) if page[0] == slug)
    adjacent = []
    for offset, label in [(-1, "Previous"), (1, "Next")]:
        index = page_index + offset
        if 0 <= index < len(PAGES):
            name, page_title, _ = PAGES[index]
            adjacent.append(f'<a class="page-{label.lower()}" href="../{name}/"><span>{label}</span>{html.escape(page_title)}</a>')
    pager = '<nav class="page-navigation" aria-label="Guide pages">' + "".join(adjacent) + '</nav>'
    return f'''<!doctype html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html.escape(title)} — CRISP</title>
<meta name="description" content="{html.escape(description, quote=True)}">
<link rel="icon" href="../../favicon.svg?v=gripper-symmetric" type="image/svg+xml">
<link rel="stylesheet" href="../../styles.css?v=contact-footer">
</head><body class="docs-page docs-{html.escape(slug)}">
<a class="skip" href="#main">Skip to content</a>
{render_header('../../', active='publications' if slug == 'publications' else 'documentation')}
<div class="docs-layout wrap">
<aside class="docs-nav"><details open><summary>Guide contents</summary><nav aria-label="Documentation">{nav}</nav></details></aside>
<main id="main" class="prose">{content}{pager}</main>
<aside class="docs-toc"><p class="eyebrow">On this page</p>{toc}</aside>
</div>
<footer class="footer docs-footer wrap"><p>CRISP · Interactive &amp; Networked Robotics Laboratory</p><div class="footer-links"><a href="mailto:hopelee@snu.ac.kr" aria-label="Contact CRISP by email">Contact</a><a href="https://github.com/INRoL/crisp/blob/main/docs/{slug}.md">View Markdown ↗</a></div></footer>
</body></html>'''


def validate(directory=OUT):
    pages = {}
    for path in directory.rglob("*.html"):
        parser = References()
        parser.feed(path.read_text(encoding="utf-8"))
        pages[path.resolve()] = parser
    for path, parser in pages.items():
        for value in parser.urls:
            url = urlsplit(value)
            if url.scheme or url.netloc:
                continue
            target = (path.parent / unquote(url.path)).resolve() if url.path else path
            if target.is_dir():
                target /= "index.html"
            if not target.is_relative_to(directory.resolve()) or not target.is_file():
                raise ValueError(f"{path.relative_to(directory)}: missing local resource {value}")
            if url.fragment and target in pages and unquote(url.fragment) not in pages[target].ids:
                raise ValueError(f"{path.relative_to(directory)}: missing anchor {value}")


def build():
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir()
    landing = (SITE_SRC / "index.html").read_text(encoding="utf-8")
    landing = re.sub(r'<header class="header wrap">.*?</header>', render_header(), landing, count=1, flags=re.S)
    refs = References()
    refs.feed(landing)
    files = {"index.html", "styles.css", "favicon.svg", "papers/crisp-preprint.pdf"}
    for value in refs.urls:
        url = urlsplit(value)
        if url.scheme or url.netloc or not url.path or url.path in {"./", "/"}:
            continue
        if url.path.startswith("docs/"):
            continue
        files.add(unquote(url.path))
    for name in sorted(files):
        source = (SITE_SRC / name).resolve()
        if not source.is_relative_to(SITE_SRC.resolve()) or not source.is_file():
            raise ValueError(f"Invalid or missing site asset: {name}")
        target = OUT / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    (OUT / "index.html").write_text(landing, encoding="utf-8")
    for slug, title, description in PAGES:
        target = OUT / "docs" / slug / "index.html"
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(render_doc(slug, title, description), encoding="utf-8")
    (OUT / ".nojekyll").write_text("", encoding="utf-8")
    validate()
    print(f"Built homepage and {len(PAGES)} documentation pages at {OUT}")


if __name__ == "__main__":
    build()
