This repository contains research and implementation of Kalman filters applied to IMU data, developed at the University of Bristol. Instructions for how to access this are written below.

## Contents

**LabBook/**: This contains a series of markdown files that explain the theory and implementation of the Kalman filter in various scenarios, including examples with velocity from position, battery output, and IMU data processing.

**Code**: Modular Python scripts 1-9, discussed in the Jupyter Book.

**Slides/**: Presentation slides made using marp

## Getting started

To read the files as a Jupyter book, you can use the following command in the terminal:

```bash
jupyter-book build LabBook/
```

You may need to install jupyter-book first if you haven't already:

```bash
pip install jupyter-book
```

Additionally, the Jupyter Book can be viewed online at [https://malachihibbins.github.io/IMU/](https://malachihibbins.github.io/IMU/).

The Jupyterbook HTML files can be found in the `_build/html` directory after building the book. You can open the `index.html` file in a web browser to view the book.

A PDF version of the book is also available in the `_build/pdf` directory. Although there are known formatting issues.
