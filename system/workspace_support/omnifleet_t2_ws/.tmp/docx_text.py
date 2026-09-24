import zipfile, re, sys
path = "/home/iecme/workspace/omnifleet_t2_ws/.dsh-im/inbound/20260907-170334-JprkOp/01-北理工问题梳理0511_3_.docx"
z = zipfile.ZipFile(path)
names = [n for n in z.namelist() if n.startswith("word/") and n.endswith(".xml")]
media = [n for n in z.namelist() if n.startswith("word/media/")]
print(f"[media files: {len(media)}]", file=sys.stderr)
xml = z.read("word/document.xml").decode("utf-8", "ignore")
# paragraph split
paras = re.split(r"</w:p>", xml)
out = []
for p in paras:
    # keep only text runs
    texts = re.findall(r"<w:t[^>]*>(.*?)</w:t>", p, re.S)
    line = "".join(texts)
    line = (line.replace("&amp;", "&").replace("&lt;", "<").replace("&gt;", ">")
                .replace("&quot;", '"').replace("&apos;", "'"))
    out.append(line)
text = "\n".join(out)
text = re.sub(r"\n{3,}", "\n\n", text)
print(text)
