{{- define "rails.fullname" -}}
{{- template "imjacinta.fullname" . -}}-{{- .name -}}
{{- end -}}

{{- define "rails.name" -}}
{{- template "imjacinta.name" . -}}-{{- .name -}}
{{- end -}}

{{- define "rails.container.spec" -}}
image: {{ .valspec.image.name }}:{{ .valspec.image.tag }}
envFrom:
- configMapRef:
    name: {{ template "imjacinta.fullname" . }}
env:
- name: SECRET_KEY_BASE
  valueFrom:
    secretKeyRef:
      name: {{ template "rails.fullname" . }}
      key: secret_key_base
- name: RAILS_MASTER_KEY
  valueFrom:
    secretKeyRef:
      name: {{ template "rails.fullname" . }}
      key: master_key
- name: POSTGRES_PASSWORD
  valueFrom:
    secretKeyRef:
      name: {{ .Release.Name }}-{{ .Values.db.secret.name }}
      key: {{ .Values.db.secret.key }}
- name: GCS_CREDS
  valueFrom:
    secretKeyRef:
      name: {{ template "rails.fullname" . }}
      key: gcs_creds_json
- name: REDIS_URL
  value: "redis://{{ include "call-nested" (list . "redis" "redis.fullname") }}-master:6379/{{ .valspec.redis_db | default 0 }}"
- name: MAILER_PASSWORD
  valueFrom:
    secretKeyRef:
      name: {{ template "rails.fullname" . }}
      key: mailer_apikey
{{- with .valspec.container }}
{{- toYaml . | nindent 0 }}
{{- end -}}
{{- end -}}