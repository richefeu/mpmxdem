#!/bin/sh
# Compile les figures de structure (PDF autonomes, insérables dans mpmbox_doc.tex)
cd "$(dirname "$0")" || exit 1
for f in fig*.tex; do
  echo "--- $f"
  pdflatex -interaction=batchmode "$f" >/dev/null || echo "ECHEC: $f"
done
rm -f *.aux *.log
echo "PDF générés :"
ls -1 fig*.pdf
