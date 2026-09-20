# SSH access to the navigation Pi

The deployment tools connect over SSH. Replace `hao@hao.local` below with your board's username and hostname or IP address.

## Connect

```bash
ssh hao@hao.local
```

For key-based access, create a dedicated key if needed, then install its public key on the board:

```bash
ssh-keygen -t ed25519 -f ~/.ssh/goosetriever
ssh-copy-id -i ~/.ssh/goosetriever.pub hao@hao.local
ssh-add ~/.ssh/goosetriever
```

Keep the private key on your development machine. If `ssh-copy-id` is unavailable, add the contents of the `.pub` file to the board account's `~/.ssh/authorized_keys`. Configure SSH to select the key by adding this to your local `~/.ssh/config`:

```sshconfig
Host hao.local
    User hao
    IdentityFile ~/.ssh/goosetriever
    IdentitiesOnly yes
```

Confirm that unattended SSH works after loading the key into your SSH agent:

```bash
ssh -o BatchMode=yes hao@hao.local 'python3 --version'
```

## Diagnose and deploy

From the repository root:

```bash
bash tools/diagnostics/pi_doctor.sh --host hao.local
bash tools/deploy/nav_pi.sh --help
```

Initial provisioning runs on the Pi with administrative privileges; review [the provisioning script](../../hardware/nav_pi/setup.sh) before running it. The deployed checkout remains `~/retriever`, and the service is named `retriever-bridge.service`.

## Earlier setup helpers

The original Windows and Python setup scripts are preserved in [`tools/legacy/ssh/`](../../tools/legacy/ssh/). They were written for an earlier board setup and modify key protection, sudo access, and optional login settings. They are not required for the development workflow above.
