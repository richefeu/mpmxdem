set bmargin 6
set lmargin 12
set rmargin 12
#set key lm t
plot 'Work.txt' u 1:($2+$3+$4+$5) w l linewidth 2 lc rgb 'red' title '-W_n - W_t + W_p + W_{int}' ,\
'Work.txt' u 1:6 w l linewidth 1 title 'ΔE_c'

set xlabel font "Helvetica,20"
set ylabel font "Helvetica,20"
set title font "Helvetica,20"
set key font "Helvetica,15"
set xrange [* : *] noreverse nowriteback
set xlabel 't'
set ylabel 'E'
