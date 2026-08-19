#!/usr/bin/env python3
"""Assemble le Print Checker et toutes ses dependances en UN fichier HTML autonome.

Reutilise la technique de c98 (DELIGNY_R_D_PORTAL/deploy/bundle-single-file.py)
sans reutiliser son code : la liste d'actifs de c98 est codee en dur pour
nano-worlds, et surtout nano-worlds vendorise deja Three.js alors que le Print
Checker le tire d'un CDN. Ce qui est repris, c'est le principe : servir les
modules depuis des octets embarques plutot que du reseau, SANS reecrire un seul
specificateur d'import.

Strategie retenue ici, plus simple que le patch de window.fetch : chaque module
devient une URL blob:, et une carte d'import generee a l'execution fait
correspondre les specificateurs nus ("three", "three/addons/loaders/X.js") a ces
blobs. Le script principal est lui-meme charge en blob par import() dynamique,
APRES injection de la carte — c'est la seule contrainte d'ordre a respecter.

Usage : python3 bundle-hors-ligne.py [SORTIE]   (defaut : print-checker-hors-ligne.html)
"""
import json, os, re, sys

RACINE = os.path.dirname(os.path.abspath(__file__))
SORTIE = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RACINE, "print-checker-hors-ligne.html")

def lire(rel):
    with open(os.path.join(RACINE, rel), encoding="utf-8") as f:
        return f.read()

html = lire("index.html")

# 1. le script principal (type="module") et la carte d'import d'origine
m_script = re.search(r'<script type="module">(.*?)</script>', html, re.S)
if not m_script: sys.exit("script module introuvable")
principal = m_script.group(1)

# 2. modules embarques : les 7 chargeurs importent "three", jamais l'inverse
MODULES = {
    "three": "vendor/three.js",
    "three/addons/loaders/OBJLoader.js": "vendor/OBJLoader.js",
    "three/addons/loaders/PLYLoader.js": "vendor/PLYLoader.js",
    "three/addons/loaders/3MFLoader.js": "vendor/3MFLoader.js",
    "three/addons/loaders/AMFLoader.js": "vendor/AMFLoader.js",
    "three/addons/loaders/FBXLoader.js": "vendor/FBXLoader.js",
    "three/addons/loaders/ColladaLoader.js": "vendor/ColladaLoader.js",
    "three/addons/loaders/GLTFLoader.js": "vendor/GLTFLoader.js",
    "./geometrie.js": "geometrie.js",
}
sources = {spec: lire(chemin) for spec, chemin in MODULES.items()}
sources["__principal__"] = principal

amorce = """
(function(){
  // Ordre imperatif : creer les blobs, PUIS injecter la carte d'import, PUIS
  // seulement importer. Une carte posee apres le premier import ne sert plus.
  var SRC = __SOURCES__;
  var url = {};
  function blob(code){ return URL.createObjectURL(new Blob([code], {type:'text/javascript'})); }
  // "three" d'abord : les chargeurs le reclament par son nom.
  url['three'] = blob(SRC['three']);
  var carte = { imports: { 'three': url['three'] } };
  Object.keys(SRC).forEach(function(spec){
    if (spec === 'three' || spec === '__principal__') return;
    var code = SRC[spec].replace(/from\\s*(["'])three\\1/g, 'from "' + url['three'] + '"');
    url[spec] = blob(code);
    carte.imports[spec] = url[spec];
  });
  var s = document.createElement('script');
  s.type = 'importmap';
  s.textContent = JSON.stringify(carte);
  document.head.appendChild(s);
  var p = SRC['__principal__']
      .replace(/from\\s*(["'])three\\1/g, 'from "' + url['three'] + '"')
      .replace(/from\\s*(["'])three\\/addons\\/loaders\\/([^"']+)\\1/g,
               function(_, q, f){ return 'from "' + carte.imports['three/addons/loaders/' + f] + '"'; })
      .replace(/from\\s*(["'])\\.\\/geometrie\\.js\\1/g, 'from "' + carte.imports['./geometrie.js'] + '"');
  import(blob(p)).catch(function(e){
    document.body.insertAdjacentHTML('afterbegin',
      '<pre style="padding:16px;color:#b00">Chargement hors ligne impossible : ' + e + '</pre>');
  });
})();
"""
amorce = amorce.replace("__SOURCES__", json.dumps(sources, ensure_ascii=False))

# 3. on retire l'importmap CDN et le script module, on pose l'amorce classique
html = re.sub(r'<script type="importmap">.*?</script>\s*', '', html, flags=re.S)
html = html.replace(m_script.group(0), '<script>' + amorce + '</script>')

with open(SORTIE, "w", encoding="utf-8") as f:
    f.write(html)
print(f"{SORTIE} — {os.path.getsize(SORTIE)//1024} Ko")
