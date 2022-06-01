// My override
void dorkit(PubMaster& pm, MessageBuilder& msg, cereal::ModelDataV2::Builder &framed, const ModelOutput& net_outputs);
// My fill function
void fill_model(cereal::ModelDataV2::Builder &framed, const ModelOutput &net_outputs);