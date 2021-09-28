# Documentation

----------------

This folder contains more extensive documentation regarding the project. It has been written using [``sphinx``](https://www.sphinx-doc.org/en/master/index.html) and can be read in the browser by simply opening ``Documentation.html``. 

## Building the documentation

To build the documentation, [``sphinx``](https://www.sphinx-doc.org/en/master/index.html) has to be installed. Do this by running 
```shell
$ pip install -U sphinx
```
Then run
``` shell
$ make html
```
to build the documentation in the folder ``build``. Finally, the docs can be opened with ``Documentation.html``. To remove/clean the documentation run
```shell
$ make clean
```

