#!/usr/bin/env bash

set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_NAME="lerobot-xense"
DEVICE="cpu"

DO_INSTALL_MINIFORGE=0
DO_SUBMODULES=1
DO_ENV=1
DO_INSTALL=1
DO_VERIFY=1
DO_TEST=0
DO_E2E=0
DO_FORK=0
DO_PUSH=0
DO_PR=0

FORK_REMOTE="fork"
BASE_REMOTE="origin"
BASE_BRANCH="main"
BRANCH_NAME=""
PR_TITLE=""
PR_BODY_FILE=""
YES=0

log() {
    printf '\n==> %s\n' "$*"
}

warn() {
    printf 'WARN: %s\n' "$*" >&2
}

die() {
    printf 'ERROR: %s\n' "$*" >&2
    exit 1
}

usage() {
    cat <<'USAGE'
Usage:
  scripts/bootstrap.sh [options]

Default:
  Initialize submodules, create/refresh the lerobot-xense mamba env, install the
  project + hardware SDK bindings, and run import verification.

Common options:
  --all                  Bootstrap, run tests, push branch, and create a PR.
  --env-name NAME        Conda/mamba environment name. Default: lerobot-xense.
  --install-miniforge    Install Miniforge first if mamba is not available.
  --test                 Run the local test suite after install.
  --e2e                  Run Makefile end-to-end smoke tests after install.
  --device DEVICE        Device passed to Makefile e2e tests. Default: cpu.
  --no-submodules        Skip git submodule initialization.
  --no-env               Skip environment creation.
  --no-install           Skip setup_env.sh --install.
  --no-verify            Skip import verification.

Fork / PR options:
  --fork                 Create/sync a GitHub fork with gh, then add remote.
  --push                 Push the current branch to the fork remote.
  --pr                   Create a pull request with gh.
  --branch NAME          Branch name to push. Defaults to current branch.
  --base-remote NAME     Upstream/base remote for PR. Default: origin.
  --fork-remote NAME     Local fork remote name. Default: fork.
  --base-branch NAME     PR base branch. Default: main.
  --pr-title TITLE       Pull request title. Defaults to latest commit subject.
  --pr-body-file PATH    Pull request body markdown file.
  -y, --yes              Do not prompt before gh fork/push/PR actions.
  -h, --help             Show this help.

Examples:
  scripts/bootstrap.sh
  scripts/bootstrap.sh --install-miniforge --test
  scripts/bootstrap.sh --all --branch fix/taccap-serial
USAGE
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --all)
                DO_TEST=1
                DO_FORK=1
                DO_PUSH=1
                DO_PR=1
                ;;
            --env-name)
                [[ $# -ge 2 ]] || die "--env-name requires a value"
                ENV_NAME="$2"
                shift
                ;;
            --install-miniforge)
                DO_INSTALL_MINIFORGE=1
                ;;
            --test)
                DO_TEST=1
                ;;
            --e2e)
                DO_E2E=1
                ;;
            --device)
                [[ $# -ge 2 ]] || die "--device requires a value"
                DEVICE="$2"
                shift
                ;;
            --no-submodules)
                DO_SUBMODULES=0
                ;;
            --no-env)
                DO_ENV=0
                ;;
            --no-install)
                DO_INSTALL=0
                ;;
            --no-verify)
                DO_VERIFY=0
                ;;
            --fork)
                DO_FORK=1
                ;;
            --push)
                DO_PUSH=1
                ;;
            --pr)
                DO_PR=1
                ;;
            --branch)
                [[ $# -ge 2 ]] || die "--branch requires a value"
                BRANCH_NAME="$2"
                shift
                ;;
            --base-remote)
                [[ $# -ge 2 ]] || die "--base-remote requires a value"
                BASE_REMOTE="$2"
                shift
                ;;
            --fork-remote)
                [[ $# -ge 2 ]] || die "--fork-remote requires a value"
                FORK_REMOTE="$2"
                shift
                ;;
            --base-branch)
                [[ $# -ge 2 ]] || die "--base-branch requires a value"
                BASE_BRANCH="$2"
                shift
                ;;
            --pr-title)
                [[ $# -ge 2 ]] || die "--pr-title requires a value"
                PR_TITLE="$2"
                shift
                ;;
            --pr-body-file)
                [[ $# -ge 2 ]] || die "--pr-body-file requires a value"
                PR_BODY_FILE="$2"
                shift
                ;;
            -y|--yes)
                YES=1
                ;;
            -h|--help)
                usage
                exit 0
                ;;
            *)
                die "unknown option: $1"
                ;;
        esac
        shift
    done
}

confirm() {
    local prompt=$1

    if [[ "$YES" -eq 1 ]]; then
        return 0
    fi

    read -r -p "$prompt [y/N] " reply
    [[ "$reply" =~ ^[Yy]$ ]]
}

have_command() {
    command -v "$1" >/dev/null 2>&1
}

source_conda() {
    local base

    if base="$(find_conda_base 2>/dev/null)"; then
        # shellcheck source=/dev/null
        source "$base/etc/profile.d/conda.sh"
        return 0
    fi

    return 1
}

find_conda_base() {
    local base
    local candidates=(
        "$HOME/miniforge3"
        "$HOME/mambaforge"
        "$HOME/miniconda3"
        "$HOME/anaconda3"
    )

    for base in "${candidates[@]}"; do
        if [[ -f "$base/etc/profile.d/conda.sh" ]]; then
            printf '%s\n' "$base"
            return 0
        fi
    done

    if have_command conda; then
        base="$(conda info --base 2>/dev/null || true)"
        if [[ -n "$base" && -f "$base/etc/profile.d/conda.sh" ]]; then
            printf '%s\n' "$base"
            return 0
        fi
    fi

    if have_command mamba; then
        base="$(mamba info --base 2>/dev/null || true)"
        if [[ -n "$base" && -f "$base/etc/profile.d/conda.sh" ]]; then
            printf '%s\n' "$base"
            return 0
        fi
    fi

    return 1
}

env_create_flag() {
    local base
    base="$(find_conda_base)" || return 1

    if [[ "$base" == "$HOME/miniforge3" || "$base" == "$HOME/mambaforge" ]] || have_command mamba; then
        printf '%s\n' "--mamba"
    else
        printf '%s\n' "--conda"
    fi
}

env_exists() {
    source_conda || return 1
    conda env list | awk '{print $1}' | grep -Fxq "$ENV_NAME"
}

current_branch() {
    git -C "$PROJECT_ROOT" branch --show-current
}

ensure_branch() {
    if [[ -z "$BRANCH_NAME" ]]; then
        BRANCH_NAME="$(current_branch)"
    fi

    if [[ -z "$BRANCH_NAME" ]]; then
        die "detached HEAD: pass --branch NAME before using --push or --pr"
    fi
}

remote_repo_slug() {
    local remote=$1
    local url

    url="$(git -C "$PROJECT_ROOT" remote get-url "$remote")" || return 1
    case "$url" in
        git@github.com:*)
            url="${url#git@github.com:}"
            ;;
        https://github.com/*)
            url="${url#https://github.com/}"
            ;;
        ssh://git@github.com/*)
            url="${url#ssh://git@github.com/}"
            ;;
        *)
            return 1
            ;;
    esac
    url="${url%.git}"
    printf '%s\n' "$url"
}

ensure_miniforge() {
    if find_conda_base >/dev/null 2>&1; then
        return 0
    fi

    [[ "$DO_INSTALL_MINIFORGE" -eq 1 ]] || die "mamba/conda not found. Install Miniforge or rerun with --install-miniforge."

    log "Installing Miniforge"
    local installer="$PROJECT_ROOT/Miniforge3-$(uname)-$(uname -m).sh"
    if [[ ! -f "$installer" ]]; then
        have_command curl || die "curl is required to download Miniforge"
        curl -fL "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh" -o "$installer"
    fi
    bash "$installer" -b -p "$HOME/miniforge3"
}

init_submodules() {
    [[ "$DO_SUBMODULES" -eq 1 ]] || return 0

    log "Initializing git submodules"
    git -C "$PROJECT_ROOT" submodule update --init --recursive --progress
}

create_env() {
    [[ "$DO_ENV" -eq 1 ]] || return 0

    ensure_miniforge
    source_conda || die "could not source conda.sh after Miniforge/conda detection"

    if env_exists; then
        log "Using existing conda environment: $ENV_NAME"
        return 0
    fi

    local create_flag
    create_flag="$(env_create_flag)" || die "could not determine conda/mamba environment manager"

    log "Creating conda environment: $ENV_NAME"
    bash "$PROJECT_ROOT/setup_env.sh" "$create_flag" "$ENV_NAME"
}

run_in_env() {
    local base

    base="$(find_conda_base)" || die "conda base not found"
    "$base/bin/conda" run -n "$ENV_NAME" "$@"
}

install_project() {
    [[ "$DO_INSTALL" -eq 1 ]] || return 0

    log "Installing project and hardware SDK bindings"
    run_in_env bash "$PROJECT_ROOT/setup_env.sh" --install
}

verify_imports() {
    [[ "$DO_VERIFY" -eq 1 ]] || return 0

    log "Verifying Python imports"
    run_in_env python - <<'PY'
checks = [
    ("lerobot", "import lerobot; print(lerobot.__file__)"),
    ("xensevr_pc_service_sdk", "import xensevr_pc_service_sdk; print(xensevr_pc_service_sdk.__file__)"),
    ("xensesdk", "import xensesdk; print(xensesdk.__file__)"),
    ("xense.taccap", "import xense.taccap; print(xense.taccap.__file__)"),
    ("torchcodec", "import torchcodec; print(torchcodec.__version__)"),
]

failed = False
for name, code in checks:
    try:
        namespace = {}
        exec(code, namespace)
        print(f"[OK] {name}")
    except Exception as exc:
        failed = True
        print(f"[FAIL] {name}: {exc!r}")

raise SystemExit(1 if failed else 0)
PY
}

run_tests() {
    [[ "$DO_TEST" -eq 1 ]] || return 0

    log "Running pytest"
    run_in_env pytest -q tests
}

run_e2e() {
    [[ "$DO_E2E" -eq 1 ]] || return 0

    log "Running end-to-end smoke tests"
    run_in_env make -C "$PROJECT_ROOT" DEVICE="$DEVICE" test-end-to-end
}

ensure_gh() {
    have_command gh || die "GitHub CLI (gh) is required for --fork, --push, or --pr"
    gh auth status >/dev/null 2>&1 || die "gh is not authenticated. Run: gh auth login"
}

ensure_fork() {
    [[ "$DO_FORK" -eq 1 ]] || return 0

    ensure_gh
    local base_repo
    base_repo="$(remote_repo_slug "$BASE_REMOTE")" || die "could not resolve GitHub repo from remote '$BASE_REMOTE'"

    log "Creating or syncing GitHub fork"
    if confirm "Create/sync your fork and add remote '$FORK_REMOTE'?"; then
        gh repo fork "$base_repo" --remote=false --default-branch-only
        local fork_url repo_name viewer
        repo_name="${base_repo#*/}"
        viewer="$(gh api user -q .login)"
        fork_url="git@github.com:${viewer}/${repo_name}.git"
        if git -C "$PROJECT_ROOT" remote get-url "$FORK_REMOTE" >/dev/null 2>&1; then
            git -C "$PROJECT_ROOT" remote set-url "$FORK_REMOTE" "$fork_url"
        else
            git -C "$PROJECT_ROOT" remote add "$FORK_REMOTE" "$fork_url"
        fi
        git -C "$PROJECT_ROOT" fetch "$FORK_REMOTE"
    else
        die "fork step cancelled"
    fi
}

push_branch() {
    [[ "$DO_PUSH" -eq 1 ]] || return 0

    ensure_gh
    ensure_branch

    if ! git -C "$PROJECT_ROOT" remote get-url "$FORK_REMOTE" >/dev/null 2>&1; then
        die "remote '$FORK_REMOTE' does not exist. Run with --fork or add it manually."
    fi

    if [[ -n "$(git -C "$PROJECT_ROOT" status --porcelain)" ]]; then
        warn "working tree has uncommitted changes; commit or stash them before pushing for PR review"
    fi

    log "Pushing branch '$BRANCH_NAME' to '$FORK_REMOTE'"
    if confirm "Push '$BRANCH_NAME' to '$FORK_REMOTE'?"; then
        git -C "$PROJECT_ROOT" push -u "$FORK_REMOTE" "$BRANCH_NAME"
    else
        die "push step cancelled"
    fi
}

create_pr() {
    [[ "$DO_PR" -eq 1 ]] || return 0

    ensure_gh
    ensure_branch

    local base_repo
    base_repo="$(remote_repo_slug "$BASE_REMOTE")" || die "could not resolve GitHub repo from remote '$BASE_REMOTE'"

    local head_ref="$BRANCH_NAME"
    local fork_repo
    if fork_repo="$(remote_repo_slug "$FORK_REMOTE" 2>/dev/null)"; then
        head_ref="${fork_repo%%/*}:$BRANCH_NAME"
    fi

    local args=(
        pr create
        --repo "$base_repo"
        --base "$BASE_BRANCH"
        --head "$head_ref"
    )

    if [[ -n "$PR_TITLE" ]]; then
        args+=(--title "$PR_TITLE")
    elif git -C "$PROJECT_ROOT" rev-parse --verify HEAD >/dev/null 2>&1; then
        args+=(--title "$(git -C "$PROJECT_ROOT" log -1 --pretty=%s)")
    fi

    if [[ -n "$PR_BODY_FILE" ]]; then
        [[ -f "$PR_BODY_FILE" ]] || die "PR body file not found: $PR_BODY_FILE"
        args+=(--body-file "$PR_BODY_FILE")
    else
        args+=(--fill)
    fi

    log "Creating pull request"
    if confirm "Create PR from '$BRANCH_NAME' into '$BASE_BRANCH'?"; then
        gh "${args[@]}"
    else
        die "PR step cancelled"
    fi
}

main() {
    parse_args "$@"
    cd "$PROJECT_ROOT"

    init_submodules
    create_env
    install_project
    verify_imports
    run_tests
    run_e2e
    ensure_fork
    push_branch
    create_pr

    log "Done"
}

main "$@"
