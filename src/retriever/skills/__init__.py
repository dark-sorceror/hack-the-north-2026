"""Skills: deterministic control loops with timeouts, each returning a Result.

The planner decides WHICH skill runs next; a skill decides HOW. Nothing in this
package calls a language model.
"""

from retriever.skills.library import DispatchRecord, Skill, SkillLibrary, result_to_json

__all__ = ["DispatchRecord", "Skill", "SkillLibrary", "result_to_json"]
