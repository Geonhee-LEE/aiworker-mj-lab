# Decisions (ADR-lite)

_prepend, 단조 증가 D-NNN. 사소한 변경에는 발급하지 않는다._

## D-001 — 2026-08-30 — TODO 권위를 Notion 대신 TODO.md 파일 단독으로 둔다

- **Context**: Representation-Aware-MPPI는 Notion DB를 canonical로 쓰고 TODO.md를
  미러로 둔다. 이 프로젝트는 외부 서비스 의존을 줄이고 싶다는 사용자 요청이 있었다.
- **Decision**: `TODO.md` 파일 자체가 유일한 권위다. `scripts/todo_tool.py`가
  기계적 수정을 담당해 마크다운 표 손상을 막는다.
- **Alternatives**: (a) Notion DB + 미러 그대로 이식 (b) GitHub Issues를 canonical로
- **Status**: accepted
- **Refs**: 부트스트랩 커밋, `docs/prd.md` R-F-008
