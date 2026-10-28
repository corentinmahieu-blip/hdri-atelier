from pathlib import Path
import re,json
root=Path(__file__).resolve().parent
source=root/'src'
strip=lambda s: re.sub(r'^import .*?;\s*$', '', re.sub(r'\bexport (?=(?:const|function|class|async))','',s), flags=re.M)
worker=strip((source/'registration.mjs').read_text())+'\n'+strip((source/'mosaic.mjs').read_text())+'\n'+(source/'worker.js').read_text()
bundle='\'use strict\';\nconst WORKER_SOURCE='+json.dumps(worker)+';\n'
for name in ['math.mjs','registration.mjs','mosaic.mjs','storage.mjs','capture.mjs','guidance.mjs','session-import.mjs','viewer.mjs','zip.mjs','app.mjs']:
    bundle+=strip((source/name).read_text())+'\n'
html=(source/'shell.html').read_text().replace('/* STYLE */',(source/'style.css').read_text()).replace('/* BUNDLE */',bundle)
(root/'index.html').write_text(html)
print('index.html autonome généré :',len(html.encode()),'octets')

final=strip((source/'registration.mjs').read_text())+'\n'+strip((source/'mosaic.mjs').read_text())+'\n'+strip((source/'final-engine.mjs').read_text())+'\n'+(source/'final-worker.js').read_text()
(root/'final-worker.js').write_text(final)
print('final-worker.js généré :',len(final.encode()),'octets')
