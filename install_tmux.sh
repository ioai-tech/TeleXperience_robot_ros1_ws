#!/bin/bash

# Function to check if tmux is installed
check_tmux_installed() {
    if command -v tmux &>/dev/null; then
        echo "tmux is already installed"
    else
        echo "tmux is not installed. Installing tmux..."
        install_tmux
    fi
}

# Function to install tmux on Ubuntu
install_tmux() {
    sudo apt-get update
    sudo apt-get install -y tmux
    sudo apt-get install -y xclip
}

# Function to enable mouse support in tmux
enable_tmux_mouse_support() {
    TMUX_CONF="$HOME/.tmux.conf"
    touch $TMUX_CONF
    
    # Enable mouse support
    if ! grep -q "set -g mouse on" "$TMUX_CONF"; then
        echo "Enabling mouse support in tmux..."
        echo "set -g mouse on" >> "$TMUX_CONF"
    fi

    # Enable Vi mode for copy mode
    if ! grep -q "setw -g mode-keys vi" "$TMUX_CONF"; then
        echo "Enabling Vi mode for copy mode..."
        echo "setw -g mode-keys vi" >> "$TMUX_CONF"
    fi

    # Set the default shell to bash
    if ! grep -q "set-option -g default-shell /bin/bash" "$TMUX_CONF"; then
        echo "Setting the default shell to bash..."
        echo "set-option -g default-shell /bin/bash" >> "$TMUX_CONF"
    fi

    # Bind 'Ctrl+Shift+C' to copy selected text to system clipboard
    if ! grep -q "bind -T copy-mode-vi C-S-c send-keys -X copy-pipe-and-cancel \"xclip -selection clipboard -i\"" "$TMUX_CONF"; then
        echo "Binding 'Ctrl+Shift+C' to copy selected text to system clipboard..."
        echo "bind -T copy-mode-vi C-S-c send-keys -X copy-pipe-and-cancel \"xclip -selection clipboard -i\"" >> "$TMUX_CONF"
    fi

    # Reload tmux configuration if tmux is running
    if tmux info &>/dev/null; then
        tmux source-file "$TMUX_CONF"
    fi
}

# Check if tmux is installed
check_tmux_installed

# Enable mouse support in tmux
enable_tmux_mouse_support
