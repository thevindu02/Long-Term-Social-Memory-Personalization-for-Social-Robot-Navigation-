#!/bin/bash
while true; do
    sleep 300
    if ! git diff-index --quiet HEAD -- || [ -n "$(git ls-files --others --exclude-standard)" ]; then
        echo "Changes detected! Auto-backing up to GitHub..."
        git add .
        git commit -m "Auto-backup: $(date +'%Y-%m-%d %H:%M:%S')"
        git push origin main
        echo "Backup complete!"
    fi
done
