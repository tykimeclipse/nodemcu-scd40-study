-- ================================================
--  study_env_logs 테이블 생성
--  NodeMCU-02: SCD40 + BH1750 + LD2410
-- ================================================

CREATE TABLE IF NOT EXISTS study_env_logs (
  id          bigserial    PRIMARY KEY,
  created_at  timestamptz  NOT NULL DEFAULT now(),
  device_id   text         NOT NULL DEFAULT 'nodemcu-02',
  temperature numeric(4,1) CHECK (temperature BETWEEN -10 AND 60),
  humidity    numeric(4,1) CHECK (humidity    BETWEEN 0   AND 100),
  co2         integer      CHECK (co2         BETWEEN 400 AND 5000),
  lux         numeric(8,1) CHECK (lux         >= 0),
  occupied    boolean,
  distance    integer      CHECK (distance    BETWEEN 0   AND 600)
);

-- 시계열 조회 최적화 인덱스
CREATE INDEX IF NOT EXISTS idx_study_env_created_at
  ON study_env_logs (created_at DESC);

CREATE INDEX IF NOT EXISTS idx_study_env_device_id
  ON study_env_logs (device_id, created_at DESC);

-- Realtime 활성화
-- INSERT 위주 append-only 테이블이므로 DEFAULT로 충분
ALTER TABLE study_env_logs REPLICA IDENTITY DEFAULT;

-- RLS 활성화 + INSERT 전용 policy
ALTER TABLE study_env_logs ENABLE ROW LEVEL SECURITY;

-- anon 키로 INSERT만 허용 (SELECT/UPDATE/DELETE 차단)
CREATE POLICY "anon insert only"
  ON study_env_logs
  FOR INSERT
  TO anon
  WITH CHECK (true);

-- 대시보드(브라우저)에서 SELECT도 필요하므로 아래도 같이 실행
CREATE POLICY "anon select"
  ON study_env_logs
  FOR SELECT
  TO anon
  USING (true);
