# Second task Jacobian

Given:

1. $ z= \begin{bmatrix} 0&0&1 \end{bmatrix}^T$
2. $p = p_5(q) - p_4(q)$
3. $d_l(q) = \frac{p_z(q)}{\parallel p(q) \parallel}$
4. $\phi(q) = \text{arccos}(d_l(q))$

We can compute $J_2$ as: 
$$
\newcommand{\diff}[2]{\frac{\partial {#1}}{\partial {#2}}}
\newcommand{\norm}[1]{\parallel {#1} \parallel}
\dot{\phi}= \diff{\text{arccos}(d_l)}{d_l}\diff{d_l}{q}\dot{q}\\
\diff{\text{arccos}(d_l)}{d_l} = \frac{-1}{\sqrt{1-d_l^2}}\\
\diff{d_l}{q} = \frac{\dot{p_z}\norm{p} - p_z\dot{\norm{p}}}{\norm{p}^2}\\
\left\{
\begin{align*}
\dot{p_z} &= \diff{p_{z}}{q} = \hat{z}^T\begin{bmatrix}
J_5 - J_4
\end{bmatrix} \\
\dot{\norm{p}} &= \diff{(\norm{p})}{q}= \frac{p_{z}p^T}{\norm{p}}\begin{bmatrix}
J_5-J_4
\end{bmatrix}
\end{align*}\\
\right.
\dot{\phi} = (\frac{\hat{z}^T\norm{p}^2 - p_{z}p^T}{\norm{p}^3})\begin{bmatrix}
J_5 - J_4
\end{bmatrix}\dot{q}\\
J_2 = (\frac{\hat{z}^T\norm{p}^2 - p_{z}p^T}{\norm{p}^3})\begin{bmatrix}
J_5 - J_4
\end{bmatrix}
$$


