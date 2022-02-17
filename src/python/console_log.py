
import click

def print_log(line, fg=None, bg=None, bold=False, underline=False, reverse=False, nl=True):
    click.secho(line, fg=fg, bg=bg, bold=bold, underline=underline, reverse=reverse, nl=nl)

def print_header(line, nl=True):
    print_log("")
    print_log(line, reverse=True, nl=nl)

def print_info(line, bold=True, underline=False):
    click.secho(line, fg="green", bold=bold, underline=underline)

def print_warning(line, bold=False, underline=False):
    click.secho(line, fg="yellow", bold=bold, underline=underline)

def print_error(line, bold=True, underline=False):
    click.secho(line, fg="red", bold=bold, underline=underline)

debug_enabled = False
def print_debug(line, nl=True):
    global debug_enabled
    if debug_enabled:
        print_log(line, nl=nl)

def set_debug(value = True):
    global debug_enabled
    debug_enabled = bool(value)
