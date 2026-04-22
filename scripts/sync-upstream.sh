#!/usr/bin/env bash
# Syncs changes from upstream (jstanhope3/hri-safe-remote-control-system) into a
# review branch targeting our 'noetic' branch.
#
# Usage:
#   ./scripts/sync-upstream.sh          # dry run: show what would happen
#   ./scripts/sync-upstream.sh --run    # execute the sync
#
# Background:
#   This repo is a private mirror of jstanhope3/hri-safe-remote-control-system,
#   itself a fork of humanisticrobotics/hri-safe-remote-control-system.
#   Our working branch is 'noetic' (ROS Noetic port); upstream only has 'master'.
#
# What --run does:
#   1. Ensures the 'upstream' remote is configured.
#   2. Fetches from upstream.
#   3. Creates a dated branch off 'noetic'.
#   4. Attempts a merge of upstream/master.
#      - If clean: pushes the branch and opens (or links) a PR toward 'noetic'.
#      - If conflicts: aborts and lists the conflicting files so the developer
#        can resolve them locally before re-running with --run.
set -euo pipefail

UPSTREAM_REMOTE="upstream"
UPSTREAM_URL="https://github.com/jstanhope3/hri-safe-remote-control-system.git"
BASE_BRANCH="noetic"
ORIGIN_REPO="Greenzie/hri-safe-remote-control-system"
SYNC_BRANCH="upstream-sync-$(date +%Y%m%d)"
DRY_RUN=true

for arg in "$@"; do
    case "$arg" in
        --run) DRY_RUN=false ;;
        *) echo "error: unknown argument '$arg'" >&2; exit 1 ;;
    esac
done

# Print a command; execute it only when not in dry-run mode.
step() {
    echo "+ $*"
    if [ "$DRY_RUN" = false ]; then
        "$@"
    fi
}

# --- 1. Ensure upstream remote exists ---
if ! git remote get-url "$UPSTREAM_REMOTE" &>/dev/null; then
    step git remote add "$UPSTREAM_REMOTE" "$UPSTREAM_URL"
fi

# --- 2. Fetch upstream (always, so the comparison below is current) ---
echo "Fetching $UPSTREAM_REMOTE..."
if [ "$DRY_RUN" = false ]; then
    git fetch "$UPSTREAM_REMOTE" --quiet
else
    git fetch "$UPSTREAM_REMOTE" --quiet 2>/dev/null || true
fi

# --- 3. Check for new commits ---
new_commits=$(git log --oneline "$UPSTREAM_REMOTE/master" ^"$BASE_BRANCH")
if [ -z "$new_commits" ]; then
    echo "$BASE_BRANCH is already up to date with $UPSTREAM_REMOTE/master."
    exit 0
fi

count=$(echo "$new_commits" | wc -l)
echo ""
echo "$count new commit(s) in $UPSTREAM_REMOTE/master not present in $BASE_BRANCH:"
echo "$new_commits"
echo ""

if [ "$DRY_RUN" = true ]; then
    echo "Dry run -- would execute:"
    echo ""
    step git checkout -b "$SYNC_BRANCH" "$BASE_BRANCH"
    step git merge "$UPSTREAM_REMOTE/master"
    step git push origin "$SYNC_BRANCH"
    echo ""
    echo "Re-run with --run to execute."
    exit 0
fi

# --- 4. Create sync branch and attempt merge ---
git checkout -b "$SYNC_BRANCH" "$BASE_BRANCH"
echo "+ git merge $UPSTREAM_REMOTE/master"

if ! git merge "$UPSTREAM_REMOTE/master" --no-commit --no-ff 2>/dev/null; then
    conflicting=$(git diff --name-only --diff-filter=U)
    echo ""
    echo "Merge has conflicts in the following file(s):"
    while IFS= read -r f; do printf '  %s\n' "$f"; done <<< "$conflicting"
    echo ""
    echo "Resolve the conflicts locally, then:"
    echo "  git add <files>"
    echo "  git merge --continue"
    echo "  git push origin $SYNC_BRANCH"
    git merge --abort
    git checkout "$BASE_BRANCH"
    git branch -D "$SYNC_BRANCH"
    exit 1
fi

# No conflicts -- commit and push.
git commit --no-edit
echo "+ git push origin $SYNC_BRANCH"
git push origin "$SYNC_BRANCH"

# --- 5. Open or link PR ---
echo ""
if command -v gh &>/dev/null; then
    gh pr create \
        --repo "$ORIGIN_REPO" \
        --base "$BASE_BRANCH" \
        --head "$SYNC_BRANCH" \
        --title "Sync upstream ($count commit(s))" \
        --body "$(printf "Automated upstream sync from \`%s/master\`.\n\n## Commits\n\n\`\`\`\n%s\n\`\`\`" "$UPSTREAM_REMOTE" "$new_commits")"
else
    echo "gh not found. Open the PR at:"
    echo "  https://github.com/$ORIGIN_REPO/compare/$BASE_BRANCH...$SYNC_BRANCH"
fi
