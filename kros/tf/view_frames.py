from .buffer import Buffer


def view_frames(buffer: Buffer) -> str:
    """좌표계 트리를 들여쓴 글로 그린다 (ROS의 view_frames가 만드는 그림의 글 버전). 부모가 없는 좌표계가 뿌리다."""
    children: dict[str | None, list[str]] = {}
    for frame in buffer.all_frames():
        children.setdefault(buffer.parent_of(frame), []).append(frame)
    lines: list[str] = []

    def walk(frame: str, depth: int) -> None:
        lines.append("  " * depth + frame)
        for child in children.get(frame, []):
            walk(child, depth + 1)

    for root in children.get(None, []):
        walk(root, 0)
    return "\n".join(lines) + ("\n" if lines else "")
