#!/usr/bin/env bash

# Function to get Ubuntu distribution codename
get_ubuntu_codename() {
    source /etc/os-release
    echo $UBUNTU_CODENAME
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
    local source_line="$3$shell_type"

    if ! grep -qF "$source_line" "$file"; then
        echo "Adding the following line to your shells config file: $file"
        echo "$source_line"
        echo "$source_line" >> "$file"
    fi
}