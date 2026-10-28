"""Assemble un panorama SDR à partir des photos capturées (sans prétention HDR)."""
from pathlib import Path
import sys
import cv2

def main(folder):
    folder=Path(folder)
    photos=sorted(folder.glob('capture_*.jpg'))
    # V1 : bande horizontale seulement. Les vues plafond/sol sont conservées pour traitement ultérieur.
    photos=[p for p in photos if 1<=int(p.stem.split('_')[-1])<=12]
    if len(photos)<3:
        raise SystemExit('Il faut au moins trois photos horizontales.')
    images=[]
    for p in photos:
        im=cv2.imread(str(p))
        if im is None: continue
        images.append(im)
    stitcher=cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
    status,pano=stitcher.stitch(images)
    if status!=cv2.Stitcher_OK:
        raise SystemExit(f'Assemblage impossible (code {status}). Augmente le recouvrement et évite la parallaxe. Les photos originales sont conservées.')
    out=folder/'panorama_SDR.jpg'
    cv2.imwrite(str(out),pano,[cv2.IMWRITE_JPEG_QUALITY,95])
    print(f'Panorama SDR créé : {out}')
    print('ATTENTION : image panoramique partielle, non équirectangulaire 360x180 garantie, et NON HDRI. Ne pas convertir artificiellement en EXR HDR.')
if __name__=='__main__':
    main(sys.argv[1] if len(sys.argv)>1 else '.')
