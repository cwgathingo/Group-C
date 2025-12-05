# Coding Standard

## Introduction
This standard defines how your code should be styled for assignments and projects. Following these conventions produces code that is more readable and easier to follow across languages (examples are in Python). Writing clean, legible code also improves employability and can reduce written report length by capturing relevant information in code.

## Identifiers
### Variables / Attributes
- Use meaningful names and include units when appropriate.
- Start with a lowercase letter and use camelCase (capital letter for each new word).
- Examples: `weightInKilos`, `heightInMetres`, `delayInMilliseconds`.

### Class Identifiers
- Start with an uppercase letter and use nouns.
- Examples: `Person`, `Doctor`, `Appointment`.

### Constants
- Use uppercase with underscores between words unless a lowercase form is required by convention (e.g., `g` vs `G`).
- Examples: `PI = 3.149265445`, `COLUMNS = 10`.

## Method / Function Names
- Start with a lowercase letter and follow camelCase.
- Names should be verbs.
- Examples: `add(a, b)`, `checkTwoEqual(array)`.

## Comments
- Include comments that explain the intent of self-contained code sections and outline the algorithm used.
- Aim for self-documenting code; simplify or refactor if code is hard to follow.
- Place comments before the code they describe and wrap long comments across lines.

### Pydoc
- In Python, use the Pydoc format so documentation can be generated automatically.

### Class Comments
- Begin each class with a preamble describing its purpose, stored data, and provided services to help users understand how to use it quickly.

### Method / Function Comments
- Comment public methods at minimum, documenting inputs and return values.

Example:
```python
"""
Calculates the factorial of a number
@param n: Input integer to take the factorial of
@return: Factorial of input
"""
def factorial(n):
    result = 1
    for i in range(1, n + 1):
        result *= i
    return result
```

## Use of Online Resources and Textbooks
If you use specific ideas or code fragments from external sources, cite them precisely and indicate which parts of the code are based on them.
- Textbooks: reference specific pages or figures.
- Online forums (e.g., Stack Overflow): link to the exact answer.
- AI tools (e.g., ChatGPT, GitHub Copilot): include the prompts used.
- No reference is needed for sources used only to understand a language construct.

Examples:
```
The code for the following function has been taken from terminus:
Extracting minimum and maximum x-values Python, Stack Exchange Network,
11 Dec 2010. https://stackoverflow.com/a/4416448 [accessed 14 Jan 2025].
User contributions licensed under CC BY-SA 3.0.
```
```
The code for the following function has been obtained from
ChatGPT: Write a Python function that takes an array of strings as
argument and returns a sorted array as result but do not use the
built-in sort function. OpenAI, ChatGPT based on GPT-4.
https://chat.openai.com/chat [accessed 14 Jan 2025].
```

## Indentation and Braces
- Indent consistently to convey program structure. Python uses indentation to define blocks.
- Use spaces or tabs, but do not mix them in a project/file; PEP 8 recommends 4 spaces per level.
- Inconsistent indentation raises `IndentationError`.
- Indent bodies of `if`, `for`, `while`, `def`, `class`, etc.

Example:
```python
# If-else example
x = 10
if x > 5:
    print("x is greater than 5")
else:
    print("x is 5 or less")

def greet(name):
    print("Hello, " + name)
    if name == "Alice":
        print("Welcome back, Alice!")
    else:
        print("Welcome, guest!")

greet("Alice")
```

## Python PEP Standards Cheat Sheet

| Section | Key Rules |
| --- | --- |
| Code Layout | 4 spaces per indent (no tabs); max 79 chars/line (72 for docstrings); use blank lines to separate sections |
| Naming Conventions | Modules/packages: lowercase with underscores; Classes/exceptions: PascalCase; Functions/variables: lowercase with underscores; Constants: ALL CAPS; Private: `_var`; Special: `__init__`, `__str__` |
| Imports | One import per line; order: stdlib, third-party, local; avoid wildcard imports |
| Whitespace | Spaces around operators (`x = y + 1`); no extra spaces inside (), [], {}; do not align with spaces; use indentation |
| Strings & Docstrings | Be consistent with ' or "; triple quotes for docstrings; docstrings describe purpose, args, return |
| Comments | Keep up to date; use full sentences; explain why, not what |
| Programming | Use `is` for None checks; do not compare to True/False; prefer for-loops and comprehensions |
| Type Hints | Use PEP 484 hints, e.g., `def add(x: int, y: int) -> int:` |
| Other PEPs | PEP 20 (Zen of Python); PEP 257 (Docstrings); PEP 484 (Type hints); PEP 572 (Walrus operator) |

## Conclusion
Following this guide makes code easier to understand, debug, and maintain while helping keep written reports concise by embedding relevant information directly in code.
