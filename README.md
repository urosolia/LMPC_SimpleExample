\documentclass{article}
\usepackage[utf8]{inputenc}
\usepackage{amsmath} % assumes amsmath package installed
\usepackage{amssymb}  % assumes amsmath package installed
\usepackage{graphicx}
\usepackage{amsmath}
\usepackage{epstopdf}
\usepackage{mathtools}
\usepackage{listings}
\usepackage{dsfont}
\usepackage{url}
\usepackage[ruled,norelsize]{algorithm2e}
\usepackage{color}
\usepackage{booktabs} % nicer tables :-)
\usepackage{multirow}   % afor multi-row tables
\usepackage{balance}    % have even columns on last page
\usepackage{soul}   % used to cross out text
\usepackage{hyperref}   % \ref{}, \cite{}  clickeable, load hyperref after cite for clickeable citations
\usepackage[noadjust]{cite} % multiple citations are combined
\newcommand*{\QEDA}{\hfill\ensuremath{\blacksquare}}%
\newcommand*{\QEDB}{\hfill\ensuremath{\square}}%

\makeatletter
\newcommand{\removelatexerror}{\let\@latex@error\@gobble}
\makeatother

\newcommand*{\red}[1]{\textcolor{red}{#1}}
\newcommand*{\blue}[1]{\textcolor{blue}{#1}}
\newcommand*{\replace}[2]{\textcolor{red}{\st{#1}}\ \textcolor{blue}{#2}}	% \replace{old text}{new text}%
\newcommand*{\soutred}[1]{\textcolor{red}{\st{#1}}} 

\newcommand{\tKinf}{\mathcal{K}_{\infty}}%Maximum Robust Attractive Set
\newcommand{\tCinf}{\mathcal{C}_{\infty}}%Maximum Robust Control Invariant Set
\newcommand{\tOinf}{\mathcal{O}_\infty}  %maximum robust admissible set
\newcommand{\tO}{\mathcal{O}}
\newcommand{\tK}{\mathcal{K}}
\newcommand{\Kc}{\mathcal{K}_{c}}
\newcommand{\CC}{\mathcal{C}}
\newcommand{\Xf}{\mathcal{X}_N}                  %Finite time feasible
\newcommand{\tKN}{\mathcal{K}_{N}}         %Maximum Attractive Set
\DeclareMathOperator{\Reach}{Reach}
\DeclareMathOperator{\Pre}{Pre}
\DeclareMathOperator{\Int}{Int}
\DeclareMathOperator{\Succ}{Succ}
\newcommand{\beq}{\begin{equation}}
\newcommand{\eeq}{\end{equation}}
\newcommand{\rr}{{\mathbb R}}
\newcommand{\zz}{{\mathbb Z}}
\newcommand{\cvd}{\hfill$\blacksquare$\vspace{0em}\par\noindent}
\newcommand{\argminF}{\mathop{\mathrm{argmin}}\limits}   % ASdeL
\usepackage{bm}

\newcounter{algorithmctr}[section]
\renewcommand{\thealgorithmctr}{\thesection.\arabic{algorithmctr}}
\newenvironment{algdesc}%
   {\refstepcounter{algorithmctr}\begin{list}{}{%
       \setlength{\rightmargin}{0\linewidth}%
       \setlength{\leftmargin}{.05\linewidth}
        \setlength{\itemsep}{1pt}
  \setlength{\parskip}{0pt}
  \setlength{\parsep}{0pt}}%
       \rmfamily\small
       \item[]{\setlength{\parskip}{0ex}\hrulefill\par%
        \nopagebreak{\bfseries\textsf{Algorithm \thealgorithmctr~}}}}%
   {{\setlength{\parskip}{-1ex}\nopagebreak\par\hrulefill} \end{list}}
\newtheorem{assumption}{Assumption}
\newtheorem{theorem}{Theorem}
\newtheorem{proposition}{Proposition}
\newtheorem{lemma}{Lemma}
\newtheorem{remark}{Remark}
\newtheorem{corollary}{Corollary}
\newtheorem{definition}{Definition}
\newenvironment{denselist}{\begin{list}{}{
        \setlength{\itemsep}{1pt}
  \setlength{\parskip}{0pt}
  \setlength{\parsep}{0pt}}}{\end{list}}
  
\title{LMPC Cookbook}
\date{November 2018}

\begin{document}

\maketitle

\section{Preliminaries}
The Learning Model Predictive Controller (LMPC) is a control framework designed for iterative tasks. At each $j$th ``iteration" or ``trial" of the control task, we assume to store the closed-loop trajectory and the associated input sequence,
\begin{equation*}
\begin{aligned}
    {\bf{x}}^j &= [x_0^j, \ldots, x_{T^j}^j]\\
    {\bf{u}}^j &= [u_0^j, \ldots, u_{T^j-1}^j]\\
\end{aligned}
\end{equation*}
where $T^j$ is the time at which the task is completed.

The key idea is to use the stored data to compute a convex safe set and an approximation to the value function. The convex safe set is defined as the convex all of the union of the stored data,
\begin{equation}\label{eq:convSS}
    \mathcal{CS}^j = \text{Conv}\big(\bigcup_{i = 0}^{j} \bigcup_{t = 0}^{T^j} x_k^i \big) = \{x \in \mathbb{R}^n : \exists \lambda_k^i \geq 0, \sum_{i = 0}^{j} \sum_{t = 0}^{T^j} \lambda_k^i = 1, \sum_{i = 0}^{j} \sum_{t = 0}^{T^j} \lambda_k^i x_k^i= x\}.
\end{equation}
Now, recall the definition of the cost-to-go associated with the stored state $x_k^i$,
\begin{equation}\label{eq:costToGo}
    Q_k^j = h(x_k^j, u_k^j) + Q_{k+1}^j
\end{equation}
for  $Q_{T^j}^j = h(x_{T^j}^j, 0), \forall i \geq 0$. The above cost-to-go of the stored trajectories is used to define the $Q$-function
\begin{equation}\label{eq:termCost}
    \begin{aligned}
        Q^{j,*}(x) = \min_{\lambda_{k}^j \in [0, 1]} & \quad \sum_{i=0}^j \sum_{k=0}^{T^i} Q_k^i \lambda_k^i\\
\text{ s.t } & \quad \sum_{i=0}^j \sum_{k=0}^{T^i} x_k^i \lambda_k^i = x,~\quad \sum_{i=0}^j \sum_{k=0}^{T^i} \lambda_k^i = 1,
    \end{aligned}
\end{equation}
which will be used as a terminal cost in the Finite Time Optimal Control Problem (FTOCP) solved by the LMPC.

\section{LMPC Implementation}
The safe set and function approximations described in the previous Section are used defined the LMPC. At each time $t$ of the $j$th iteration we solve the following finite time optimal control problem
\begin{subequations}\label{eq:RelaxedFTOCP}
	\begin{align}
    J_{t\rightarrow t+N}^{\scalebox{0.4}{LMPC},j}(x_t^j) =\min_{{\bf{U}}_t^j, \boldsymbol{\lambda}_t^j} & \quad \bigg[  \sum_{k=t}^{t+N-1}  h(x_{k|t}^j,u_{k|t}^j) + \sum_{i=0}^{j-1} \sum_{k=0}^{T^i} Q_k^i \lambda_k^i\bigg] \label{eq:cost}\\
	\text{s.t.}& \quad x_{t|t}^j=x_t^j, \label{eq:RelaxedFTOCP_IC}\\
    & \quad x_{k+1|t}^j=f(x_{k|t}^j, u_{k|t}^j), \quad ~\forall k = t, \cdots, t+N-1\label{eq:RelaxedFTOCP_Dyn} \\
	& \quad x_{k|t}^j \in \mathcal{X}, u_{k|t}^j \in \mathcal{U}, \quad \quad ~~ \forall k = t, \cdots, t+N-1 \label{eq:RelaxedFTOCP_Cons}\\ 
	& \quad \lambda_k^i \geq 0, \sum_{i = 0}^{j-1} \sum_{t = 0}^{T^j} \lambda_k^i = 1, \sum_{i = 0}^{j-1} \sum_{t = 0}^{T^j} \lambda_k^i x_k^i= x_{t+N|t} \label{eq:RelaxedFTOCP_Term}
	\end{align}
\end{subequations}
where ${\bf{U}}_t^j = [u_{t|t}^j, \ldots, u_{t+N-1|t}^j]$ is the vector of open loop sequences and the vector ${\boldsymbol{\lambda}_t^j} \in \mathbb{R}^{\Pi_i^{j-1}T^i}$ collects the multiplies associated with each recorded data. Equation~\eqref{eq:cost} represents the cost function which is the summation of the running cost $h(\cdot, \cdot)$ and the approximation to the value function $Q^{*,j-1}(\cdot)$ defined in \eqref{eq:termCost}. Equations~\eqref{eq:RelaxedFTOCP_IC}-\eqref{eq:RelaxedFTOCP_Dyn} and \eqref{eq:RelaxedFTOCP_Cons} represent the initial condition, dynamics constraint, state and input constraints, respectively. Finally, equation~\eqref{eq:RelaxedFTOCP_Term} enforces the latest predicted state $x_{t+N|t}$ into the convex hull of the recorded states. Let 
\begin{equation*}
    {\bf{U}}_t^{j,*} = [u_{t|t}^{j,*}, \ldots, u_{t+N-1|t}^{j,*}]
\end{equation*}
be the optimal solution to \eqref{eq:RelaxedFTOCP} at time $t$ of the $j$th iteration, we apply to the system the first element of the optimizer vector
\begin{equation*}
    u_t = u_{t|t}^{j,*}
\end{equation*}
The finite time optimal control problem \eqref{eq:RelaxedFTOCP} is repeated at time $t+1$, based on the new state $x_{t+1|t+1} = x_{t+1}^j$, until the iteration is completed.

\section{Example}
On my GitHub page () I have uploaded a simple Matlab Code (based on YALMIP https://yalmip.github.io/), where the LMPC is used to solve the following infinite time constrained optimal control problem
\begin{equation*}
    \begin{aligned}
        J_{0\rightarrow \infty}^{*}(x_S)=\min_{u_0, u_1,\ldots} &\quad \sum\limits_{k=0}^{\infty} x_{k}^\top Q x_k + u_{k}^\top R u_k\\
\textrm{s.t. }
& \quad x_{k+1}=A x_{k}+B u_{k},~\forall k\geq 0\\
   & \quad x_{k} \in \mathcal{X},~u_k \in \mathcal{U},~\forall k\geq 0\\
   & \quad x_{0}=x_S.
    \end{aligned}
\end{equation*}

List of files in the GitHub Repo:
\begin{itemize}
    \item \textit{Main.m}: This files first loads the system matrices and the data from a first feasible solution. Afterwards executes the LMPC controller (\textit{LMPC.m}) and plots the results.
    \item \textit{LMPC.m}: This files runs the LMPC controller for \#\textit{Iterations}. At each time $t$ of the $j$th iteration the FTOCP \eqref{eq:RelaxedFTOCP} (\textit{FTOCP.m}) is solved. Furthermore, when $j$th iteration is completed the closed-loop data are used to build the safe set and approximation to the value function. 
    \item \textit{FTOCP.m}: This files solves the FTOCP \eqref{eq:RelaxedFTOCP} using the data from the previous iterations to compute the convex safe set and the of the cost-to-go associated with the recorded data, which is compute in \textit{ComputeCost.m}
    \item \textit{ComputeCost.m}: This file computes the cost-to-go associated with the recorded data.
\end{itemize}

\section{References}
This short guide illustrated how to implemented the LMPC proposed in:
\begin{itemize}
    \item Ugo Rosolia and Francesco Borrelli. "Learning Model Predictive Control for Iterative Tasks. A Data-Driven Control Framework." In IEEE Transactions on Automatic Control (2018).
    \item Ugo Rosolia and Francesco Borrelli. "Learning Model Predictive Control for Iterative Tasks: A Computationally Efficient Approach for Linear System." IFAC-PapersOnLine 50.1 (2017).
\end{itemize}
\end{document}
