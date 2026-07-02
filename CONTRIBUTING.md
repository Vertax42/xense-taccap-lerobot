# Contributing to LeRobot-Xense

Thanks for your interest in contributing! **LeRobot-Xense** is a downstream fork
of [🤗 LeRobot](https://github.com/huggingface/lerobot), scoped to a single
device: the **TacCap-Gripper** (TacCap = *Tactile Capture* Gripper) — a handheld
**UMI** leader gripper for tactile data collection — together with the **Pico4**
teleoperator/tracker and Xense tactile cameras.

## Scope — what belongs here

Please keep contributions focused on this repository's purpose:

- The TacCap-Gripper devices (`src/lerobot/robots/taccap_gripper`,
  `bi_taccap_gripper`) and their serial/USB auto-discovery.
- The Pico4 teleop/tracker (`src/lerobot/teleoperators/pico4`, `bi_pico4`).
- Xense tactile cameras (`src/lerobot/cameras/xense`).
- Recording / dataset / setup tooling that supports the above.

For changes to the **generic LeRobot framework** (datasets, policies, training,
other robots), please contribute to
[upstream LeRobot](https://github.com/huggingface/lerobot) instead — this fork
tracks upstream and intentionally stays slim.

## Ways to contribute

- **Report bugs** or request features via the
  [issue tracker](https://github.com/Vertax42/xense-taccap-lerobot/issues)
  (use the bug-report template).
- **Improve docs** — the root `README.md`, the device guide
  (`src/lerobot/robots/taccap_gripper/README.md`), and docstrings.
- **Fix bugs / add features** within the scope above, with a pull request.

## Development setup

```bash
git clone --recurse-submodules https://github.com/Vertax42/xense-taccap-lerobot.git
cd xense-taccap-lerobot
bash ./setup_env.sh --mamba lerobot-xense
mamba activate lerobot-xense
bash ./setup_env.sh --install
```

See the README's **Installation** section for the hardware SDK details
(`xensesdk` installs from PyPI; the XenseVR PC Service `.deb` is fetched
out-of-band).

## Code style & checks

This repo uses [pre-commit](https://pre-commit.com) (ruff lint + format, and
other hooks). Install and run it before pushing:

```bash
pre-commit install
pre-commit run --all-files
```

The `Quality` GitHub Action runs the same checks on every push and pull request.

## Pull requests

- Branch off `main`; keep PRs focused and reasonably small.
- Use [Conventional Commits](https://www.conventionalcommits.org) for commit and
  PR titles, e.g. `fix(taccap): ...`, `feat(setup): ...`, `docs: ...`,
  `chore: ...`.
- Fill in the PR template (summary, what changed, how it was tested).
- Make sure `pre-commit run --all-files` is clean and, where applicable, that
  you have verified the change on real hardware (note this in the PR).

## Code of Conduct

By participating you agree to abide by our
[Code of Conduct](./CODE_OF_CONDUCT.md). Report unacceptable behavior to
**yaphetys@gmail.com**.

## License

By contributing, you agree that your contributions will be licensed under the
[Apache License 2.0](./LICENSE), the same license as this project and upstream
LeRobot.
