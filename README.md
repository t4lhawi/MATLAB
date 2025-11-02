# Complete MATLAB Summary & Reference Guide

> A full reference of MATLAB — from basics to advanced operations.  
> Includes syntax, examples, and formulas for students, engineers, and researchers.  
> Ideal for **Embedded Systems, Signal Processing, Control, AI, and Data Analysis**.


## Table of Contents

1. [Introduction](#1-introduction)  
2. [Basics](#2-basics)  
3. [Vectors & Matrices](#3-vectors--matrices)  
4. [Operators](#4-operators)  
5. [Control Structures](#5-control-structures)  
6. [Functions](#6-functions)  
7. [Plotting & Visualization](#7-plotting--visualization)  
8. [Mathematical Operations](#8-mathematical-operations)  
9. [Polynomials & Equations](#9-polynomials--equations)  
10. [Signal Processing](#10-signal-processing)  
11. [File Handling](#11-file-handling)  
12. [Simulink & Toolboxes](#12-simulink--toolboxes)  
13. [Tips & Shortcuts](#13-tips--shortcuts)  
14. [Resources](#14-resources)  

---

## 1. Introduction

MATLAB = **MAT**rix **LAB**oratory.  
It’s a numerical computing environment used for:
- Mathematical modeling
- Data visualization
- Signal & image processing
- Control systems
- Machine learning and AI

Run code in **Command Window** or in **.m scripts**.

---

## 2. Basics

### Variables
```matlab
a = 5;             % Scalar
b = [1 2 3];       % Row vector
c = [1; 2; 3];     % Column vector
A = [1 2; 3 4];    % Matrix
````

### Constants

```matlab
pi       % 3.1416
inf      % Infinity
NaN      % Not a Number
eps      % Floating point precision
```

### Input / Output

```matlab
x = input('Enter a number: ');
disp(x);
fprintf('Value = %.2f\n', x);
```

### Scripts vs Functions

* **Script:** sequence of commands (`myscript.m`)
* **Function:** reusable code block (`function y=f(x)`)

---

## 3. Vectors & Matrices

### Creation

```matlab
v = [1 2 3 4];
v = 1:4;                % 1 2 3 4
v = 0:2:10;             % step = 2
A = [1 2; 3 4];
zeros(2,3);  ones(3,2);
eye(3);      rand(3);   diag([1 2 3]);
```

### Indexing

```matlab
A(1,2);      % element row 1, col 2
A(:,2);      % 2nd column
A(2,:);      % 2nd row
A(1:2,1:2);  % submatrix
```

### Operations

| Operation          | Syntax              | Description                  |
| ------------------ | ------------------- | ---------------------------- |
| Addition           | `A + B`             | Matrix addition              |
| Subtraction        | `A - B`             | Matrix subtraction           |
| Multiplication     | `A * B`             | Matrix multiplication        |
| Element-wise mult. | `A .* B`            | Multiply element by element  |
| Division           | `A / B` or `A ./ B` | Matrix or element-wise       |
| Power              | `A^2` or `A.^2`     | Matrix or element-wise power |

---

## 4. Operators

| Type       | Example           | Meaning         |                    |
| ---------- | ----------------- | --------------- | ------------------ |
| Arithmetic | `+ - * / ^`       | Math operations |                    |
| Relational | `> < >= <= == ~=` | Comparisons     |                    |
| Logical    | `&                | ~ xor(a,b)`     | Logical operations |

---

## 5. Control Structures

### If / Else

```matlab
if x > 0
    disp('Positive');
elseif x < 0
    disp('Negative');
else
    disp('Zero');
end
```

### Loops

```matlab
for i = 1:5
    disp(i);
end

while x < 10
    x = x + 1;
end
```

### Switch

```matlab
switch grade
    case 'A'
        disp('Excellent');
    case 'B'
        disp('Good');
    otherwise
        disp('Invalid');
end
```

---

## 6. Functions

```matlab
function y = squareNum(x)
    y = x.^2;
end
```

Anonymous function:

```matlab
f = @(x) x.^2 + 2*x + 1;
```

---

## 7. Plotting & Visualization

### 2D Plot

```matlab
x = 0:0.1:2*pi;
y = sin(x);
plot(x, y, 'r', 'LineWidth', 2);
xlabel('x'); ylabel('sin(x)');
title('Sine Wave'); grid on;
```

### Multiple Plots

```matlab
hold on
plot(x, sin(x));
plot(x, cos(x));
legend('sin','cos');
hold off
```

### Subplots

```matlab
subplot(2,1,1); plot(x, sin(x));
subplot(2,1,2); plot(x, cos(x));
```

### 3D Plots

```matlab
[X,Y] = meshgrid(-3:0.1:3);
Z = sin(sqrt(X.^2 + Y.^2));
surf(X,Y,Z); shading interp; colorbar;
```

---

## 8. Mathematical Operations

| Function            | Description        |
| ------------------- | ------------------ |
| `sum(A)`            | Sum of elements    |
| `mean(A)`           | Average            |
| `std(A)`            | Standard deviation |
| `max(A)` / `min(A)` | Max / Min          |
| `det(A)`            | Determinant        |
| `inv(A)`            | Inverse            |
| `eig(A)`            | Eigenvalues        |
| `rank(A)`           | Matrix rank        |

---

## 9. Polynomials & Equations

### Polynomial Definition

```matlab
p = [1 3 2];   % x² + 3x + 2
roots(p)       % find roots
polyval(p, 2)  % evaluate at x=2
polyder(p)     % derivative
polyint(p)     % integral
```

### Solve Equations

```matlab
syms x
solve(x^2 + 3*x + 2 == 0, x)
diff(sin(x)*x^2)
int(exp(-x^2))
```

---

## 10. Signal Processing

### Basic Signals

```matlab
t = 0:0.01:1;
x = sin(2*pi*10*t);        % Sine wave
y = square(2*pi*5*t);      % Square wave
z = sawtooth(2*pi*5*t);    % Sawtooth
```

### Fourier Transform

```matlab
Y = fft(x);
f = (0:length(Y)-1)*Fs/length(Y);
plot(f, abs(Y));
```

### Filtering

```matlab
[b,a] = butter(3, 0.2);
y = filter(b, a, x);
```

---

## 11. File Handling

```matlab
save('data.mat','A','B');
load('data.mat');
csvwrite('data.csv',A);
B = csvread('data.csv');
```

Read/Write text:

```matlab
fid = fopen('data.txt','w');
fprintf(fid, '%f\n', A);
fclose(fid);
```

---

## 12. Simulink & Toolboxes

### Simulink

* Graphical simulation tool.
* Open with `simulink`.

### Toolboxes

| Toolbox           | Use                         |
| ----------------- | --------------------------- |
| Signal Processing | FFT, filters                |
| Control System    | PID, bode, root-locus       |
| Image Processing  | `imread`, `imshow`, filters |
| Deep Learning     | Neural networks             |
| Optimization      | Minimization, fitting       |

---

## 13. Tips & Shortcuts

| Action          | Shortcut       |
| --------------- | -------------- |
| Run script      | `F5`           |
| Clear workspace | `clear`        |
| Clear console   | `clc`          |
| Close figures   | `close all`    |
| Show variables  | `whos`         |
| Help            | `help command` |
| Documentation   | `doc command`  |

---

## 14. Resources

* [📘 MATLAB Official Docs](https://www.mathworks.com/help/matlab/)
* [🧠 MATLAB Academy](https://matlabacademy.mathworks.com/)
* [🔓 GNU Octave (Free Alternative)](https://www.gnu.org/software/octave/)
* [📺 YouTube: MATLAB Tutorials](https://www.youtube.com/c/MATLAB)

---

> 💡 “MATLAB is not just for math — it’s for thinking in matrices.”
