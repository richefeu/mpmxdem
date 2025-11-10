%%======================================================
%% doofus.cls — Custom LaTeX class
%%======================================================

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{doofus}[Custom class for code documentation (DOcumentation OFten USed)]

%% Base class
\LoadClass[a4paper,onecolumn,11pt]{report}

%%======================================================
%% Page geometry
%%======================================================
\RequirePackage[
  left=2cm,
  right=2cm,
  top=2.5cm,
  bottom=3cm,
  includehead,
  includefoot
]{geometry}


%%======================================================
%% Package setup
%%======================================================
\RequirePackage{listings}
\RequirePackage{courier}
\RequirePackage{xcolor}
\RequirePackage{graphicx}
\RequirePackage{subfigure}
\RequirePackage{bm}
\RequirePackage{hyperref}
\RequirePackage{bookmark}

%%======================================================
%% Hyperref setup
%%======================================================
\hypersetup{
  colorlinks=true,
  linkcolor=black,
  citecolor=black,
  urlcolor=black,
  pdfborderstyle={/S/U/W 0.5},
  pdfborder={0 0 1},
}

%%======================================================
%% Colors
%%======================================================
\definecolor{lightgrey}{RGB}{245,245,245}
\definecolor{lightblue}{RGB}{191,239,255}
\definecolor{codegreen}{rgb}{0,0.6,0}
\definecolor{codegray}{rgb}{0.5,0.5,0.5}
\definecolor{codepurple}{rgb}{0.58,0,0.82}
\definecolor{backcolour}{rgb}{0.95,0.95,0.92}
\definecolor{doofusblue}{RGB}{30,90,160}
\definecolor{doofusred}{RGB}{170,40,40}
\definecolor{doofusgreen}{RGB}{30,120,30}
\definecolor{doofusgray}{gray}{0.3}

%%======================================================
%% Item symbols
%%======================================================
\renewcommand{\labelitemi}{$\triangleright$}  % Level 1
\renewcommand{\labelitemii}{$\circ$}          % Level 2
\renewcommand{\labelitemiii}{$\bullet$}       % Level 3
\renewcommand{\labelitemiv}{$\ast$}           % Level 4

%%======================================================
%% Default listings configuration
%%======================================================
\lstset{
  basicstyle=\small\ttfamily,
  columns=fullflexible,
  breaklines=true,
  frame=single,
  framesep=5pt,
  framerule=0.4pt,
  backgroundcolor=\color{backcolour},
}

%%======================================================
%% Custom listing style: mystyle
%%======================================================
\lstdefinestyle{mystyle}{
  basicstyle=\small\ttfamily,
  columns=fullflexible,
  breaklines=true,
  frame=single,
  framesep=5pt,
  framerule=0.4pt,
  backgroundcolor=\color{backcolour},
  commentstyle=\color{codegreen},
  keywordstyle=\color{magenta},
  numberstyle=\tiny\color{codegray},
  stringstyle=\color{codepurple},
  basicstyle=\ttfamily\footnotesize,
  breakatwhitespace=false,
  captionpos=b,
  keepspaces=true,
  numbers=left,
  numbersep=5pt,
  showspaces=false,
  showstringspaces=false,
  showtabs=false,
  tabsize=2
}

%%------------------------------------------------------
%% C++ syntax highlighting style for listings
%%------------------------------------------------------
\definecolor{cppkeyword}{RGB}{0,0,180}
\definecolor{cppcomment}{RGB}{0,128,0}
\definecolor{cppstring}{RGB}{163,21,21}
\definecolor{cppnumber}{RGB}{128,0,128}
%\definecolor{cppbackground}{RGB}{245,245,255}

\lstdefinestyle{cppstyle}{
  language=C++,                   % enable C++ syntax rules
  backgroundcolor=\color{backcolour},
  basicstyle=\ttfamily\footnotesize,
  keywordstyle=\color{cppkeyword}\bfseries,
  commentstyle=\color{cppcomment}\itshape,
  stringstyle=\color{cppstring},
  numberstyle=\tiny\color{cppnumber},
  numbers=left,
  stepnumber=1,
  numbersep=5pt,
  showstringspaces=false,
  breaklines=true,
  frame=single,
  framerule=0.4pt,
  framesep=5pt,
  tabsize=2,
  columns=fullflexible
}

% Make cppstyle the default for all lstlisting environments
%\lstset{style=cppstyle}

%%------------------------------------------------------
%% Title page customization
%%------------------------------------------------------
\newcommand{\doofustitle}[1]{\def\@doofustitle{#1}}
\newcommand{\doofussubtitle}[1]{\def\@doofussubtitle{#1}}
\newcommand{\doofusauthor}[1]{\def\@doofusauthor{#1}}
\newcommand{\doofusdate}[1]{\def\@doofusdate{#1}}
\newcommand{\doofuslogo}[1]{\def\@doofuslogo{#1}}

%% Default values
\doofustitle{Untitled Report}
\doofussubtitle{}
\doofusauthor{Anonymous}
\doofusdate{\today}
\doofuslogo{}

%%------------------------------------------------------
%% Title page definition
%%------------------------------------------------------
\newcommand{\makedoofustitle}{%
  \begin{titlepage}
    \centering
    \vspace*{3cm}

    {\Huge\bfseries \@doofustitle \par}
    \vspace{0.8cm}
    {\Large \@doofussubtitle \par}

    \vspace{2cm}
    {\Large \@doofusauthor \par}

    \vspace{1cm}
    {\large \@doofusdate \par}

    \vspace{2cm}
    \ifx\@doofuslogo\@empty
    \else
      \includegraphics[width=0.3\textwidth]{\@doofuslogo}
    \fi

    \vfill
  \end{titlepage}
  
  % Reset counters so first section starts at 1 (no chapters)
    \setcounter{chapter}{0}
    \renewcommand{\thesection}{\arabic{section}}
    \setcounter{section}{0}
}

%%------------------------------------------------------
%% Syntax presentation commands
%%------------------------------------------------------
\newcommand{\syntax}[1]{%
  %\vspace{0.5em}%
  \noindent%
  \colorbox{backcolour}{\ttfamily\small #1}%
  %\vspace{0.5em}%
}


% Syntax component commands
\newcommand{\return}[1]{{\color{doofusblue}{\scriptsize #1}}}
\newcommand{\keyword}[1]{{\color{doofusred}\textbf{#1}}}
\newcommand{\type}[1]{{\color{doofusgreen}{\scriptsize #1}}}
\newcommand{\param}[1]{{\color{doofusgray}\textit{\textbf{#1}}}}

%%======================================================
%% End of class
%%======================================================
\endinput
