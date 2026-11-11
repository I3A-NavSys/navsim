"""Sube un repositorio grande a GitHub en varios commits/pushes, cada uno de tamaño limitado (GitHub rechaza pushes de ~2 GB).

Uso (desde la carpeta del repositorio, con la rama ya creada):
    python subir_por_lotes.py                      # reparte en lotes de 1500 MB y pregunta antes de empezar
    python subir_por_lotes.py --limit-mb 1000 --yes

- Sube los archivos NUEVOS, MODIFICADOS y BORRADOS respecto al último commit. Respeta el .gitignore (lo mismo que GitHub Desktop).
- Es REANUDABLE: si un push falla, vuelve a ejecutarlo y continúa donde se quedó.
- NO modifica ni borra ningún archivo de tu disco; solo prepara commits y hace push.
- Aborta si hay algún archivo de 100 MiB o más (GitHub lo rechazaría).
"""
import argparse, subprocess

GITHUB_FILE_LIMIT = 100 * 1024 * 1024


def git(*args, check=True, capture=True, input_bytes=None):
    r = subprocess.run(["git", *args], capture_output=capture, input=input_bytes)
    if check and r.returncode != 0:
        msg = (r.stderr or b"").decode("utf-8", "replace") if capture else ""
        raise SystemExit(f"\nERROR: git {' '.join(args)}\n{msg}")
    return r


def has_head():
    return git("rev-parse", "--verify", "-q", "HEAD", check=False).returncode == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit-mb", type=int, default=1500, help="tamaño máximo por lote (MB, sin comprimir)")
    ap.add_argument("--yes", action="store_true", help="no preguntar")
    ap.add_argument("--remote", default="origin")
    a = ap.parse_args()
    limit = a.limit_mb * 1024 * 1024

    branch = git("rev-parse", "--abbrev-ref", "HEAD").stdout.decode().strip() if has_head() else \
        git("symbolic-ref", "--short", "HEAD").stdout.decode().strip()
    print(f"Rama: {branch}")

    # 1) dejar el índice vacío (lo que haya "preparado" se volverá a preparar por lotes). No toca archivos.
    if has_head():
        git("reset", "-q")
    else:
        git("rm", "-r", "--cached", "-q", ".", check=False)

    # 2) lista de archivos pendientes: NUEVOS + MODIFICADOS + BORRADOS (respeta .gitignore)
    import os
    out = git("ls-files", "--others", "--modified", "--deleted", "--exclude-standard", "-z").stdout
    files = sorted(set(f for f in out.decode("utf-8", "surrogateescape").split("\0") if f))
    sizes = {}
    for f in files:
        try: sizes[f] = os.path.getsize(f)
        except OSError: sizes[f] = 0            # archivo borrado: no pesa, pero hay que registrar el borrado
    big = [f for f in files if sizes[f] >= GITHUB_FILE_LIMIT]
    if big:
        print("\nHay archivos de 100 MiB o más (GitHub los rechazaría). Quítalos antes de seguir:")
        for f in big: print(f"  {sizes[f] / 1048576:7.0f} MB  {f}")
        raise SystemExit(1)

    # 3) lotes
    batches, cur, cur_size = [], [], 0
    for f in files:
        if cur and cur_size + sizes[f] > limit:
            batches.append(cur); cur, cur_size = [], 0
        cur.append(f); cur_size += sizes[f]
    if cur: batches.append(cur)
    total = sum(sizes.values())
    print(f"Archivos pendientes: {len(files)} · {total / 1048576:,.0f} MB · lotes: {len(batches)} (≤ {a.limit_mb} MB cada uno)")

    # 4) si ya hay commits locales sin subir (ejecución anterior interrumpida), subirlos primero
    if has_head():
        print("\nComprobando commits locales sin subir ...")
        r = subprocess.run(["git", "push", "-u", a.remote, branch])
        if r.returncode != 0:
            raise SystemExit("\nERROR al subir los commits que ya estaban hechos. Revisa tu conexión/permisos y repite.")
    if not batches:
        print("Nada más que subir."); return
    if not a.yes and input("\n¿Continuar? [s/N] ").strip().lower() not in ("s", "si", "sí", "y", "yes"):
        raise SystemExit("Cancelado.")

    for i, batch in enumerate(batches, 1):
        size_mb = sum(sizes[f] for f in batch) / 1048576
        print(f"\n=== Lote {i}/{len(batches)}: {len(batch)} archivos, {size_mb:,.0f} MB ===")
        git("add", "-A", "--pathspec-from-file=-", "--pathspec-file-nul", input_bytes=("\0".join(batch) + "\0").encode("utf-8", "surrogateescape"))
        git("commit", "-q", "-m", f"Subida por lotes {i}/{len(batches)}")
        r = subprocess.run(["git", "push", "--progress", "-u", a.remote, branch])
        if r.returncode != 0:
            raise SystemExit(f"\nEl push del lote {i} falló. El commit está hecho en local; vuelve a ejecutar el script para reintentarlo.")
        print(f"Lote {i}/{len(batches)} subido.")
    print("\nTERMINADO: todos los archivos están en GitHub.")


if __name__ == "__main__":
    main()