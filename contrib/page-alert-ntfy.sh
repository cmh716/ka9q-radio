#!/bin/sh
# An alert for tones(1): a push notification through ntfy (https://ntfy.sh,
# the ntfy app on the phone, subscribed to the topic), with the page's
# recording attached. tones runs it once the recording is complete:
#
#   tones -t departments.csv -n Alden -c "NTFY_TOPIC=my-secret-topic ~/page-alert-ntfy.sh" fire-dispatch.local
#
# The topic is the only "password" on ntfy.sh: make it long and unguessable.
# NTFY_SERVER for a server of one's own.
: "${NTFY_TOPIC:?set NTFY_TOPIC}"
SERVER=${NTFY_SERVER:-https://ntfy.sh}
if [ -n "$PAGE_FILE" ] && [ -f "$PAGE_FILE" ]; then
  curl -sS --max-time 60 \
    -H "Title: Page: $PAGE_DEPARTMENTS" \
    -H "Tags: rotating_light" \
    -H "Priority: high" \
    -H "Filename: $(basename "$PAGE_FILE")" \
    -H "Message: $PAGE_TIME, $PAGE_FREQ MHz, $PAGE_TONES" \
    -T "$PAGE_FILE" \
    "$SERVER/$NTFY_TOPIC" > /dev/null
else
  # The recording failed (PAGE_ERROR says why): the alert, without it
  curl -sS --max-time 60 \
    -H "Title: Page: $PAGE_DEPARTMENTS" \
    -H "Tags: rotating_light" \
    -H "Priority: high" \
    -d "$PAGE_TIME, $PAGE_FREQ MHz, $PAGE_TONES (no recording: ${PAGE_ERROR:-none})" \
    "$SERVER/$NTFY_TOPIC" > /dev/null
fi
