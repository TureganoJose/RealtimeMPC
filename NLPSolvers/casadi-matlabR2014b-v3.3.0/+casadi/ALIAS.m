function v = ALIAS()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 119);
  end
  v = vInitialized;
end
