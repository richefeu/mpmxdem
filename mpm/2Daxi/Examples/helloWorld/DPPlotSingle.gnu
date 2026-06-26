set bmargin 6
set lmargin 12
set rmargin 12
#set term context {font "Helvetica,20"}
plot 'DPPlotSingle.txt' u 2:3 with points lt -1 pt 7 pointsize 1.5 lc rgb 'red' title 'meanStress',\
'DPPlotSingle.txt' u 2:3 with lines lc rgb 'red',\
'DPPlotSingle.txt' u ($2+10):($3+10):0 with labels font 'Helvetica,8' ,\
'DPPlotSingle.txt' u 2:4 w l title 'DP yield surface'
set grid back lc rgb 'black' linewidth 1.5;
set xlabel font "Helvetica,20"
set ylabel font "Helvetica,20"
set xtics 500
set ytics 500
set title font "Helvetica,20"
set xrange [* : *] noreverse nowriteback
set xlabel 'P'
set ylabel 'Q'
