NEW trajectory planning reference:
${start}
At this NEW trajectory start:
- Target relative position [forward,right,down], meters: ${target_vector_json}
- Target horizontal bearing: ${bearing}.
Positive bearing means right; negative bearing means left.
${side_rule}
This bearing describes the starting geometry, not a required dyaw_deg command.
The original directional description, such as 'front-left', identifies the object at mission start.
It does not prescribe the direction of subsequent actions.
dyaw_deg is an incremental turn relative to the preceding pose. Turns accumulate across waypoints.
For example, five actions with -8 degrees each produce a total turn of -40 degrees.
Re-evaluate the target direction after each predicted translation and turn.
Do not repeat a turn merely because the previous action turned that way.
