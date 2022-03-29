# Copyright 2005-2019 NumPy Developers.
# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This is compatibility code for older versions of numpy that lack these functions.
# The original code was copied from:
# https://github.com/numpy/numpy/blob/3dec7099ce38cb189880f6f69df318f35ff9a5ea/numpy/lib/recfunctions.py
# and then lightly edited for style.

import numpy as np


def _get_fields_and_offsets(dt, offset=0):
    # counts up elements in subarrays, including nested subarrays, and returns
    # base dtype and count
    def count_elem(dt):
        count = 1
        while dt.shape != ():
            for size in dt.shape:
                count *= size
            dt = dt.base
        return dt, count

    fields = []
    for name in dt.names:
        field = dt.fields[name]
        f_dt, f_offset = field[0], field[1]
        f_dt, n = count_elem(f_dt)

        if f_dt.names is None:
            fields.append((np.dtype((f_dt, (n,))), n, f_offset + offset))
        else:
            subfields = _get_fields_and_offsets(f_dt, f_offset + offset)
            size = f_dt.itemsize

            for i in range(n):
                if i == 0:
                    # optimization: avoid list comprehension if no subarray
                    fields.extend(subfields)
                else:
                    fields.extend([(d, c, o + i*size) for d, c, o in subfields])
    return fields


def structured_to_unstructured(arr, dtype=None, copy=False, casting='unsafe'):
    if arr.dtype.names is None:
        raise ValueError('arr must be a structured array')

    fields = _get_fields_and_offsets(arr.dtype)
    n_fields = len(fields)
    if n_fields == 0 and dtype is None:
        raise ValueError('arr has no fields. Unable to guess dtype')
    elif n_fields == 0:
        # too many bugs elsewhere for this to work now
        raise NotImplementedError('arr with no fields is not supported')

    dts, counts, offsets = zip(*fields)
    names = ['f{}'.format(n) for n in range(n_fields)]

    if dtype is None:
        out_dtype = np.result_type(*[dt.base for dt in dts])
    else:
        out_dtype = dtype

    # Use a series of views and casts to convert to an unstructured array:

    # first view using flattened fields (doesn't work for object arrays)
    # Note: dts may include a shape for subarrays
    flattened_fields = np.dtype({'names': names,
                                 'formats': dts,
                                 'offsets': offsets,
                                 'itemsize': arr.dtype.itemsize})
    arr = arr.view(flattened_fields)

    # next cast to a packed format with all fields converted to new dtype
    packed_fields = np.dtype({'names': names,
                              'formats': [(out_dtype, dt.shape) for dt in dts]})
    arr = arr.astype(packed_fields, copy=copy, casting=casting)

    # finally is it safe to view the packed fields as the unstructured type
    return arr.view((out_dtype, (sum(counts),)))


def unstructured_to_structured(arr, dtype=None, names=None, align=False,
                               copy=False, casting='unsafe'):
    if arr.shape == ():
        raise ValueError('arr must have at least one dimension')
    n_elem = arr.shape[-1]
    if n_elem == 0:
        # too many bugs elsewhere for this to work now
        raise NotImplementedError('last axis with size 0 is not supported')

    if dtype is None:
        if names is None:
            names = ['f{}'.format(n) for n in range(n_elem)]
        out_dtype = np.dtype([(n, arr.dtype) for n in names], align=align)
        fields = _get_fields_and_offsets(out_dtype)
        dts, counts, offsets = zip(*fields)
    else:
        if names is not None:
            raise ValueError("don't supply both dtype and names")
        # sanity check of the input dtype
        fields = _get_fields_and_offsets(dtype)
        if len(fields) == 0:
            dts, counts, offsets = [], [], []
        else:
            dts, counts, offsets = zip(*fields)

        if n_elem != sum(counts):
            raise ValueError('The length of the last dimension of arr must '
                             'be equal to the number of fields in dtype')
        out_dtype = dtype
        if align and not out_dtype.isalignedstruct:
            raise ValueError('align was True but dtype is not aligned')

    names = ['f{}'.format(n) for n in range(len(fields))]

    # Use a series of views and casts to convert to a structured array:

    # first view as a packed structured array of one dtype
    packed_fields = np.dtype({'names': names,
                              'formats': [(arr.dtype, dt.shape) for dt in dts]})
    arr = np.ascontiguousarray(arr).view(packed_fields)

    # next cast to an unpacked but flattened format with varied dtypes
    flattened_fields = np.dtype({'names': names,
                                 'formats': dts,
                                 'offsets': offsets,
                                 'itemsize': out_dtype.itemsize})
    arr = arr.astype(flattened_fields, copy=copy, casting=casting)

    # finally view as the final nested dtype and remove the last axis
    return arr.view(out_dtype)[..., 0]
