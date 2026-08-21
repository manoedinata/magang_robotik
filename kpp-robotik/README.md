# KPP Programming

| Key        | Value               |
| ---------- | ------------------- |
| Nama       | Hendra Manudinata   |
| NRP        | 5027251051          |
| Departemen | Teknologi Informasi |
| Fakultas   | FTEIC               |

Git repo: https://github.com/manoedinata/kpp-robotik

## About

See the [KPP instruction](https://github.com/manoedinata/kpp-robotik/blob/master/KPP%20Programming%2025.pdf) for more informations regarding the task.

## Compile

### C++ version

```
make # or mingw32-make if using MinGW on Windows
./kpp_robotik/KPPRobotik
```

Or, compile manually:

```
cd kpp_robotik/
g++ main.cpp Graph.cpp KPPRobotik.cpp -o KPPRobotik -std=c++17
./KPPRobotik
```

### Python version

```
python3 kpp.py
```

## Input

See the [input.txt](https://github.com/manoedinata/kpp-robotik/blob/master/input.txt) to test the program.
