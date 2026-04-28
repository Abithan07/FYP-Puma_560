cat > install_deps.sh << 'EOF'
#!/bin/bash

echo "====================================="
echo " Installing exact dependencies"
echo "====================================="

# All regular packages
pip install --user \
    absl-py==2.4.0 \
    aiofiles==25.1.0 \
    dill==0.4.1 \
    etils==1.13.0 \
    filelock==3.25.2 \
    flax==0.10.7 \
    fsspec==2026.2.0 \
    humanize==4.15.0 \
    importlib_resources==6.5.2 \
    jax==0.6.2 \
    jaxlib==0.6.2 \
    jmp==0.0.4 \
    joblib==1.5.3 \
    markdown-it-py==4.0.0 \
    matplotlib==3.5.1 \
    mdurl==0.1.2 \
    ml_dtypes==0.5.4 \
    msgpack==1.1.2 \
    networkx==3.4.2 \
    numpy==1.26.4 \
    opt_einsum==3.4.0 \
    optax==0.2.8 \
    orbax-checkpoint==0.11.33 \
    pandas==2.3.3 \
    Pygments==2.20.0 \
    python-dateutil==2.9.0.post0 \
    rich==14.3.3 \
    scipy==1.15.3 \
    simplejson==3.20.2 \
    tensorstore==0.1.78 \
    treescope==0.1.10 \
    uvloop==0.22.1

# Torch separately (needs --index-url)
pip install --user torch==2.1.0 \
    --index-url https://download.pytorch.org/whl/cpu

# Haiku separately (needs --no-deps)
pip install --user tabulate
pip install --user dm-haiku==0.0.16 --no-deps

echo "====================================="
echo " Verifying..."
echo "====================================="
python3 -c "import torch; print('PyTorch    :', torch.__version__)"
python3 -c "import jax; print('JAX        :', jax.__version__)"
python3 -c "import haiku as hk; print('Haiku      :', hk.__version__)"
python3 -c "import numpy as np; print('NumPy      :', np.__version__)"
python3 -c "import pandas as pd; print('Pandas     :', pd.__version__)"
python3 -c "import matplotlib; print('Matplotlib :', matplotlib.__version__)"
python3 -c "import joblib; print('Joblib     :', joblib.__version__)"
python3 -c "import dill; print('Dill       :', dill.__version__)"
echo "====================================="
echo " Done!"
echo "====================================="
EOF

chmod +x install_deps.sh
./install_deps.sh
