"""Check the binary boundary used by the GRU before any trajectory starts."""


def require_torch_numpy(torch, numpy):
    """Exercise both conversion directions instead of trusting package imports."""
    try:
        source = numpy.array([1.0, 2.0], dtype=numpy.float32)
        restored = torch.from_numpy(source).numpy()
        if not numpy.array_equal(source, restored):
            raise RuntimeError('Tensor/array roundtrip changed values')
    except Exception as error:
        raise RuntimeError(
            f'PyTorch/NumPy conversion failed (NumPy {numpy.__version__}). '
            'Run bash arm_bot/setup_runtime.sh and restart the demo.'
        ) from error
