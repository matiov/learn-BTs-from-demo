"""Script to allow dynamic remappings of namespaces."""

# Copyright (c) 2022, ABB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with
# or without modification, are permitted provided that
# the following conditions are met:
#
#   * Redistributions of source code must retain the
#     above copyright notice, this list of conditions
#     and the following disclaimer.
#   * Redistributions in binary form must reproduce the
#     above copyright notice, this list of conditions
#     and the following disclaimer in the documentation
#     and/or other materials provided with the
#     distribution.
#   * Neither the name of ABB nor the names of its
#     contributors may be used to endorse or promote
#     products derived from this software without
#     specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import tempfile


def process_templated_file(file_path: str, ros_hostname: str) -> str:
    """
    Process a templated file.

    The function will replace the following template indicators:
    - '{TEMPLATE_NAMESPACE}' with '<ros hostname>'
    - '{TEMPLATE_PREFIX}' with '<ros hostname>_'

    """
    # Use machine hostname for namespaces and transformation prefixes.
    tf_prefix = ''
    if ros_hostname != '':
        tf_prefix = ros_hostname + '_'

    # Process the templated file.
    with open(file=file_path, mode='r') as input:
        input_contents = input.read()

        output_contents = input_contents \
            .replace('{TEMPLATE_NAMESPACE}', ros_hostname) \
            .replace('{TEMPLATE_PREFIX}', tf_prefix)

        with tempfile.NamedTemporaryFile(mode='w', delete=False) as output:
            output.write(output_contents)
            return output.name
