"""Read the survey area (docs/SURVEY_AREA_FORMAT.md): a world folder / ``metadata.yaml`` or a
``blueboat_survey_area/1`` file. Read-only (CM-7); PyYAML imported lazily."""

from __future__ import annotations

from pathlib import Path

from ..core.area import AreaError, SurveyArea, area_from_dict


def load_area_file(path: str) -> SurveyArea:
    p = Path(path).expanduser()
    if p.is_dir():
        p = p / "metadata.yaml"
    if not p.is_file():
        raise AreaError(f"survey area file not found: {p}")
    import yaml  # noqa: PLC0415

    with open(p, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    return area_from_dict(data, source=str(p))
