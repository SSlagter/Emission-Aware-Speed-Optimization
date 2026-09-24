%% ICE emission parameters
% Select one engine category from EmissionsFactors.xlsx using its exact
% BuildYear and WeightClass text. Available combinations are:
%
% BuildYear                 WeightClass
% 1900 - 1974              L1 t/m L3
% 1975 - 1979              L1 t/m L3
% 1980 - 1984              L1 t/m L3
% 1985 - 1989              L1 t/m L3
% 1990 - 1994              L1 t/m L3
% 1995 - 2002              L1 t/m L3
% 2003 - 2007 CCR-1        L1 t/m L3
% 2008 - 2018 CCR-2        L1 t/m L3
% 2019 - 2019 CCR-2        L2 en L3
% 2019 – 20xx stage V      L1
% 2020 – 20xx stage V      L2 en L3
%
% Change the two selections below for another ICE use case. The selected
% row supplies base NOx and PM10 factors and the base fuel rate. The second
% workbook adjusts NOx and PM10 factors according to engine load.
% This file must run after parameter_initialisation_ship because it uses P_max.

% Sources cited for the emissions data in the supplied Excel workbooks:
% Hulskotte, J. (2018). EMS-protocol Emissies Door Binnenvaart:
% Verbrandingsmotoren. Taskgroup Traffic and Transport, TNO,
% Department of Climate Air and Sustainability, version 4.
%
% Ligterink, N., R. van Gijlswijk, G. Kadijk, R. Vermeulen,
% A. Indrajuana, M. Elstgeest, P. van Mensch, J. de Ruiter,
% R. Verbeek, J. Hulskotte, G. Geilenkirchen and M. Traa (2019).
% Emissiefactoren wegverkeer - actualisatie 2019.
% TNO report R10825v2, The Hague, Netherlands.
%
% The values used below are read from EmissionsFactors.xlsx and
% EmissionsLoadCorrections.xlsx; they are not hard-coded here.

engine_build_year = "2019 - 2019 CCR-2";
engine_weight_class = "L2 en L3";

factors = readtable("EmissionsFactors.xlsx", ...
    "VariableNamingRule", "preserve", "TextType", "string");
corrections = readtable("EmissionsLoadCorrections.xlsx", ...
    "VariableNamingRule", "preserve", "TextType", "string");

% Identify the single workbook row matching both selected fields.
row = string(factors.BuildYear) == engine_build_year & ...
      string(factors.WeightClass) == engine_weight_class;
assert(nnz(row) == 1, ...
    "Choose one matching engine age and weight class in EmissionsFactors.xlsx.");

% Workbook values may be numeric or text with a decimal comma.
number = @(v) str2double(strrep(string(v), ",", "."));

emission_param.NOx_g_per_kWh = number(factors.NOx(row));
emission_param.PM10_g_per_kWh = number(factors.PM(row));
emission_param.SFOC_base_g_per_kWh = number(factors.Fuelrate(row));

emission_param.load_percent = number(corrections.Load);
emission_param.PM10_correction = number(corrections.PM10);

% The supplied model identifies separate NOx curves for CCR-1 and CCR-2.
% For other categories, use the base NOx factor without a load correction.
if contains(engine_build_year, "CCR-1")
    emission_param.NOx_correction = number(corrections.NOx_CCR1);
elseif contains(engine_build_year, "CCR-2")
    emission_param.NOx_correction = number(corrections.NOx_CCR2);
else
    emission_param.NOx_correction = ones(height(corrections), 1);
    warning("No identified NOx load-correction curve for %s.", engine_build_year);
end

% Retain the selection with the calculated results for identification.
emission_param.engine_build_year = engine_build_year;
emission_param.engine_weight_class = engine_weight_class;

values = [emission_param.NOx_g_per_kWh; ...
          emission_param.PM10_g_per_kWh; ...
          emission_param.SFOC_base_g_per_kWh; ...
          emission_param.load_percent(:); ...
          emission_param.NOx_correction(:); ...
          emission_param.PM10_correction(:)];
assert(all(isfinite(values)), "An emissions spreadsheet value could not be read.");

% Engine rating defines load percentage. The two diesel constants convert
% calculated fuel mass to litres, then litres to operational CO2 mass.
emission_param.rated_power_kW = P_max;
emission_param.diesel_density_g_per_L = 840;
emission_param.CO2_kg_per_L = 2.68;