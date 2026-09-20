FROM python:3.12-slim
WORKDIR /app
COPY pyproject.toml ./
COPY robot_app ./robot_app
RUN pip install --no-cache-dir . && useradd --create-home robot && mkdir artifacts && chown robot artifacts
USER robot
EXPOSE 8765
CMD ["python", "-m", "robot_app.cloud", "--host", "0.0.0.0"]
