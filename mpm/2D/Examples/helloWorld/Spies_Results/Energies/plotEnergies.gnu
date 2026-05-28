set bmargin 6
set lmargin 12
set rmargin 12
#set term context {font "Helvetica,20"}
plot 'Energies.txt' u 1:2 with lines lc rgb 'red' title 'Énergie cinétique',\
'Energies.txt' u 1:3 w l lc rgb 'blue' title 'Énergie potentielle',\
'Energies.txt' u 1:4 w l lc rgb 'green' title 'Énergie élastique'
set xlabel font "Helvetica,20"
set ylabel font "Helvetica,20"
set title font "Helvetica,20"
set xrange [* : *] noreverse nowriteback
set xlabel 't'
set ylabel 'E'
