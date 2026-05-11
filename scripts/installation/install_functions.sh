#!/usr/bin/env bash

# Function to get Ubuntu distribution codename
get_ubuntu_codename() {
    source /etc/os-release

    if [ -n "${UBUNTU_CODENAME:-}" ]; then
        echo "$UBUNTU_CODENAME"
        return 0
    fi

    if [ -n "${VERSION_CODENAME:-}" ]; then
        echo "$VERSION_CODENAME"
        return 0
    fi

    case "${VERSION_ID:-}" in
        "20.04"|"20")
            echo "focal"
        ;;
        "22.04"|"22")
            echo "jammy"
        ;;
        "24.04"|"24")
            echo "noble"
        ;;
    esac
}

is_standard_ubuntu() {
    source /etc/os-release
    [ "${ID:-}" = "ubuntu" ]
}

is_ubuntu_core() {
    source /etc/os-release
    [ "${ID:-}" = "ubuntu-core" ]
}

get_supported_ros_distro() {
    case "$(get_ubuntu_codename)" in
        "focal")
            echo "foxy"
        ;;
        "jammy")
            echo "humble"
        ;;
        "noble")
            echo "jazzy"
        ;;
        *)
            return 1
        ;;
    esac
}

get_atos_venv_path() {
    echo "$HOME/.local/share/atos/venv"
}

# Function that checks if command failed
check_command_failed() {
    local exitcode="$1"
    local error_message="$2"

    if [ $exitcode -ne 0 ]; then
        echo "$error_message"
        exit 1
    fi
}

# Function to update symlink if it doesn't point to the correct location
update_symlink() {
    local target="$1"
    local link_name="$2"

    if [ -L "$link_name" ]; then
        current_target="$(readlink "$link_name")"
        if [ "$current_target" != "$target" ]; then
            rm "$link_name"
            ln -s "$target" "$link_name"
            echo "Updated symlink $link_name to $target because it pointed to $current_target previously."
        fi
    else
        ln -s "$target" "$link_name"
        echo "Created symlink $link_name to $target"
    fi
}

# Function to add the setup.bash source line if it doesn't exist
add_source_line_if_needed() {
    local file="$1"
    local shell_type="$2"
    local source_line="$3"

    if ! grep -qF "$source_line" "$file"; then
        # Ask the user only for interactive local shells.
        # In non-interactive environments (e.g. CI), append automatically.
        if [ -t 0 ] && [ -z "$DEBIAN_FRONTEND" ] && [ -z "$GITHUB_ACTION" ]; then
            echo "Do you want to add the following line to your $shell_type config file $file:"
            echo "$source_line"
            echo "y/n"
            read -r answer
            if [ "$answer" != "${answer#[Yy]}" ]; then
                echo "# Line below added by ATOS setup script" >> "$file"
                echo "$source_line" >> "$file"
            fi
        else
            echo "# Line below added by ATOS setup script" >> "$file"
            echo "$source_line" >> "$file"
        fi
    fi
}
