#!/usr/bin/env bash
# Host GUI applications must not load the Snap editor's GTK/Qt runtime.
# This is sourced by launchers; ROS/system paths and the parent terminal stay intact.
_puma_host_desktop_environment() {
    local variable value component
    local -a components kept
    for variable in \
        GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE \
        GIO_MODULE_DIR GIO_EXTRA_MODULES GI_TYPELIB_PATH \
        GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE GSETTINGS_SCHEMA_DIR \
        QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH \
        LIBGL_DRIVERS_PATH __EGL_VENDOR_LIBRARY_DIRS \
        XDG_DATA_DIRS XDG_CONFIG_DIRS LD_LIBRARY_PATH; do
        value="${!variable-}"
        [[ "$value" == */snap/* ]] || continue
        IFS=: read -r -a components <<< "$value"
        kept=()
        for component in "${components[@]}"; do
            [[ "$component" == */snap/* ]] || kept+=("$component")
        done
        if (( ${#kept[@]} )); then
            value="$(IFS=:; printf '%s' "${kept[*]}")"
            printf -v "$variable" '%s' "$value"
            export "$variable"
        else
            unset "$variable"
        fi
    done
}

_puma_host_desktop_environment
unset -f _puma_host_desktop_environment
