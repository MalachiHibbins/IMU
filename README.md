The code implementation detail and theory can be found in the LabBook folder. This contains a series of markdown files that explain the theory and implementation of the Kalman filter in various scenarios, including examples with velocity from position, battery output, and IMU data processing.

To read the files as a jupyter book, you can use the following command in the terminal:

```bash
jupyter-book build LabBook/
```

you may need to install jupyter-book first if you haven't already:

```bash
pip install jupyter-book
```

Additionally the jupyter book can be viewed online at [https://malachihibbins.github.io/IMU/](https://malachihibbins.github.io/IMU/).

The Jupyterbook HTML files can be found in the `_build/html` directory after building the book. You can open the `index.html` file in a web browser to view the book.

There is also a PDF version of the book available in the `_build/pdf` directory. Although there is formatting issues with this but works okay.