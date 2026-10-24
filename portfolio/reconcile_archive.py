"""Inspect the supplied archive without importing repositories or generated files."""
from pathlib import Path
import zipfile,hashlib,json,argparse,io,xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('archive');args=p.parse_args()
rows=[];notes=[]
with zipfile.ZipFile(args.archive) as z:
    for info in z.infolist():
        parts=info.filename.split('/')
        if info.is_dir():continue
        if any(x in parts for x in ['.git','slprj','__MACOSX']) or any(x.startswith('._') for x in parts) or info.filename.endswith(('.DS_Store','.slxc','.asv')):continue
        data=z.read(info);rel=info.filename.split('/src/',1)[-1] if '/src/' in info.filename else None
        target=root/rel if rel else None
        if target and not target.is_file() and (root/'legacy'/rel).is_file():target=root/'legacy'/rel
        row={'archive_entry':info.filename,'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),'repository_path':rel,'match':bool(target and target.is_file() and target.read_bytes()==data)}
        baseline=root/'legacy/baselines'/rel if rel else None
        compare=baseline if baseline and baseline.is_file() else target
        if compare and compare.is_file():
            original=compare.read_bytes();row['newline_normalized_match']=data.replace(b'\r\n',b'\n')==original.replace(b'\r\n',b'\n')
        if target and target.is_file() and not row['match']:row['status']='byte difference; see normalized comparison; original retained'
        if info.filename.endswith('.urdf'):
            ur=ET.fromstring(data);row['links']=len(ur.findall('link'));row['joints']=len(ur.findall('joint'))
            def signature(e):return [e.tag,dict(sorted(e.attrib.items())),[signature(c) for c in e]]
            if compare and compare.is_file():row['xml_semantic_match']=signature(ur)==signature(ET.parse(compare).getroot())
        if info.filename.endswith('.step'):
            row['header']=data[:1500].decode('ascii',errors='replace')
        if info.filename.endswith('.docx'):
            with zipfile.ZipFile(io.BytesIO(data)) as doc:
                text=' '.join(ET.fromstring(doc.read('word/document.xml')).itertext())
                (root/'outputs').mkdir(exist_ok=True);(root/'outputs/archive_report_text.txt').write_text(text,encoding='utf-8')
                row['report_text_inspected']=True
        rows.append(row)
out=root/'docs/executed';out.mkdir(exist_ok=True)
(out/'archive_reconciliation.json').write_text(json.dumps({'archive_sha256':hashlib.sha256(Path(args.archive).read_bytes()).hexdigest(),'files':rows},indent=2)+'\n')
print(json.dumps({'safe_files':len(rows),'identical_repository_files':sum(r['match'] for r in rows),'differences':[r['repository_path'] for r in rows if r.get('status')],'extras':[r['archive_entry'].split('/')[-1] for r in rows if r['repository_path'] is None]},ensure_ascii=True))
