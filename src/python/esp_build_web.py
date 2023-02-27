Import("env")
import os
import re
import io
import tempfile
import filecmp
import shutil
import fnmatch
import gzip
try:
    from minify import (html_minifier, rcssmin, rjsmin)
except ImportError:
    class Temp(object):
        def html_minify(self, x):
            return x
        def cssmin(self, x):
            return x
        def jsmin(self, x):
            return x
    html_minifier = rcssmin = rjsmin = Temp()


platform = env.get('PIOPLATFORM', '')
target_name = env['PIOENV'].upper()


def get_version():
    return '%s' % (env.get('LATEST_COMMIT'))


def get_target_name():
    return '%s' % (env.get('TARGET_NAME'))


def compress(data):
    """
    Compress data in one shot and return the compressed string.
    Optional argument is the compression level, in range of 0-9.
    """
    buf = io.BytesIO()
    with gzip.GzipFile(fileobj=buf, mode='wb', compresslevel=9, mtime=0.0) as f:
        f.write(data)
    return buf.getvalue()


def build_html(mainfile, var, out):
    with open(mainfile, 'r') as file:
        data = file.read()
        data = data.replace("{%ESP_VERSION%}", get_version())
        data = data.replace("{%TARGET_NAME%}", get_target_name())
    if mainfile.endswith('.html'):
        data = html_minifier.html_minify(data)
    if mainfile.endswith('.css'):
        data = rcssmin.cssmin(data)
    if mainfile.endswith('.js'):
        data = rjsmin.jsmin(data)
    out.write('#define %s_IMPL 1\n' % var)
    out.write('static const char PROGMEM %s[] = {\n' % var)
    out.write(','.join("0x{:02x}".format(c) for c in compress(data.encode('utf-8'))))
    out.write('\n};\n\n')


def build_common(src_dir, sources, out_file):
    fd, outpath = tempfile.mkstemp()
    print(f"Temp file {outpath}")
    with os.fdopen(fd, 'w') as out:
        out.write("#pragma once\n#include <pgmspace.h>\n\n")
        for input_f in sources:
            print(f"  parse: {input_f}")
            pieces = input_f.split(":")
            build_html(os.path.join(src_dir, pieces[0]), pieces[1], out)
    if not os.path.exists(out_file) or not filecmp.cmp(outpath, out_file):
        shutil.copyfile(outpath, out_file)
    os.remove(outpath)


def parse_html_files(env):
    src_dir = env.GetProjectOption("html_sources_dir", "").split()
    out_file = env.GetProjectOption("html_output_file", "")
    sources = env.GetProjectOption("html_sources", "").split()
    if not src_dir or not out_file or not sources:
        return
    src_dir = os.path.join(*src_dir)
    out_file = os.path.join("include", out_file)
    build_common(src_dir, sources, out_file)
    #env['BUILD_FLAGS'].append(f"-include {out_file}")


if platform in ['espressif8266', 'espressif32']:
    parse_html_files(env=env)
