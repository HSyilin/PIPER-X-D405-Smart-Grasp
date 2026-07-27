from rcl_interfaces.msg import ParameterType, ParameterValue

from smart_grasp.param_tuner import make_parameter_value, parameter_value_to_python


def test_make_parameter_value_preserves_array_types():
    doubles = make_parameter_value(ParameterType.PARAMETER_DOUBLE_ARRAY, "[0.1, 0.2]")
    integers = make_parameter_value(ParameterType.PARAMETER_INTEGER_ARRAY, "[1, 2]")
    bytes_value = make_parameter_value(ParameterType.PARAMETER_BYTE_ARRAY, "[3, 4]")
    booleans = make_parameter_value(
        ParameterType.PARAMETER_BOOL_ARRAY, "['false', 'true']"
    )

    assert list(doubles.double_array_value) == [0.1, 0.2]
    assert list(integers.integer_array_value) == [1, 2]
    assert list(bytes_value.byte_array_value) == [b"\x03", b"\x04"]
    assert list(booleans.bool_array_value) == [False, True]
    assert parameter_value_to_python(bytes_value) == [3, 4]


def test_parameter_value_to_python_uses_active_field():
    value = ParameterValue(
        type=ParameterType.PARAMETER_INTEGER_ARRAY,
        integer_array_value=[10, 20],
    )
    assert parameter_value_to_python(value) == [10, 20]


def test_parameter_value_to_python_handles_not_set():
    value = ParameterValue(type=ParameterType.PARAMETER_NOT_SET)
    assert parameter_value_to_python(value) is None
